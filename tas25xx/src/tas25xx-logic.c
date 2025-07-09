/*
 * ALSA SoC Texas Instruments TAS25XX High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2021 Texas Instruments, Inc.
 *
 * Author: Niranjan H Y, Vijeth P O
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
#include <linux/power_supply.h>
#include "inc/tas25xx-logic.h"
#include "inc/tas25xx-device.h"
#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
#include "algo/inc/tas_smart_amp_v2.h"
#include "algo/inc/tas25xx-calib.h"
#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
#include "algo/src/tas25xx-sysfs-debugfs-utils.h"
#endif
#endif /* CONFIG_TAS25XX_ALGO*/
#include "inc/tas25xx-regmap.h"
#include "inc/tas25xx-regbin-parser.h"

#ifndef DEFAULT_AMBIENT_TEMP
#define DEFAULT_AMBIENT_TEMP 20
#endif

/* 128 Register Map to be used during Register Dump*/
#define REG_CAP_MAX	128

const char *tas_power_states_str[] = {
	"TAS_POWER_ACTIVE",
	"TAS_POWER_MUTE",
	"TAS_POWER_SHUTDOWN",
};

static struct tas25xx_reg regs[REG_CAP_MAX] = {
	{0,	0},	{1,	0},	{2,	0},	{3,	0},	{4,	0},
	{5,	0},	{6,	0},	{7,	0},	{8,	0},	{9,	0},
	{10,	0},	{11,	0},	{12,	0},	{13,	0},	{14,	0},
	{15,	0},	{16,	0},	{17,	0},	{18,	0},	{19,	0},
	{20,	0},	{21,	0},	{22,	0},	{23,	0},	{24,	0},
	{25,	0},	{26,	0},	{27,	0},	{28,	0},	{29,	0},
	{30,	0},	{31,	0},	{32,	0},	{33,	0},	{34,	0},
	{35,	0},	{36,	0},	{37,	0},	{38,	0},	{39,	0},
	{40,	0},	{41,	0},	{42,	0},	{43,	0},	{44,	0},
	{45,	0},	{46,	0},	{47,	0},	{48,	0},	{49,	0},
	{50,	0},	{51,	0},	{52,	0},	{53,	0},	{54,	0},
	{55,	0},	{56,	0},	{57,	0},	{58,	0},	{59,	0},
	{60,	0},	{61,	0},	{62,	0},	{63,	0},	{64,	0},
	{65,	0},	{66,	0},	{67,	0},	{68,	0},	{69,	0},
	{70,	0},	{71,	0},	{72,	0},	{73,	0},	{74,	0},
	{75,	0},	{76,	0},	{77,	0},	{78,	0},	{79,	0},
	{80,	0},	{81,	0},	{82,	0},	{83,	0},	{84,	0},
	{85,	0},	{86,	0},	{87,	0},	{88,	0},	{89,	0},
	{90,	0},	{91,	0},	{92,	0},	{93,	0},	{94,	0},
	{95,	0},	{96,	0},	{97,	0},	{98,	0},	{99,	0},
	{100,	0},	{101,	0},	{102,	0},	{103,	0},	{104,	0},
	{105,	0},	{106,	0},	{107,	0},	{108,	0},	{109,	0},
	{110,	0},	{111,	0},	{112,	0},	{113,	0},	{114,	0},
	{115,	0},	{116,	0},	{117,	0},	{118,	0},	{119,	0},
	{120,	0},	{121,	0},	{122,	0},	{123,	0},	{124,	0},
	{125,	0},	{126,	0},	{127,	0},
};

int tas25xx_change_book(struct tas25xx_priv *p_tas25xx,
	int32_t chn, int book)
{
	int ret = -EINVAL;

	if (chn >= p_tas25xx->ch_count)
		return ret;

	ret = 0;
	if (p_tas25xx->devs[chn]->mn_current_book != book) {
		ret = p_tas25xx->plat_write(
				p_tas25xx->platform_data,
				p_tas25xx->devs[chn]->mn_addr,
				TAS25XX_BOOKCTL_PAGE, 0, chn);
		if (ret) {
			pr_err("%s, ERROR, L=%d, E=%d, ch=%d\n",
				__func__, __LINE__, ret, chn);
		} else {
			ret = p_tas25xx->plat_write(
					p_tas25xx->platform_data,
					p_tas25xx->devs[chn]->mn_addr,
					TAS25XX_BOOKCTL_REG, book, chn);
			if (ret) {
				pr_err("%s, ERROR, L=%d, E=%d, ch=%d\n",
					__func__, __LINE__, ret, chn);
			}
		}

		if (!ret)
			p_tas25xx->devs[chn]->mn_current_book = book;
	}

	return ret;
}


/* Function to Dump Registers */
void tas25xx_dump_regs(struct tas25xx_priv  *p_tas25xx, int chn)
{
	int i;
	struct linux_platform *plat_data;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	for (i = 0; i < REG_CAP_MAX; i++)
		p_tas25xx->read(p_tas25xx, chn, regs[i].reg_index, &(regs[i].reg_val));

	dev_err(plat_data->dev, "--- TAS25XX Channel-%d RegDump ---\n", chn);
	for (i = 0; i < REG_CAP_MAX/16; i++) {
		dev_err(plat_data->dev,
			"%02x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			regs[16 * i].reg_index, regs[16 * i].reg_val, regs[16 * i + 1].reg_val,
			regs[16 * i + 2].reg_val, regs[16 * i + 3].reg_val,
			regs[16 * i + 4].reg_val, regs[16 * i + 5].reg_val,
			regs[16 * i + 6].reg_val, regs[16 * i + 7].reg_val,
			regs[16 * i + 8].reg_val, regs[16 * i + 9].reg_val,
			regs[16 * i + 10].reg_val, regs[16 * i + 11].reg_val,
			regs[16 * i + 12].reg_val, regs[16 * i + 13].reg_val,
			regs[16 * i + 14].reg_val, regs[16 * i + 15].reg_val);
	}
	if (REG_CAP_MAX % 16) {
		for (i = 16 * (REG_CAP_MAX / 16); i < REG_CAP_MAX; i++)
			dev_err(plat_data->dev, "%02x: %02x\n", regs[i].reg_index,
				regs[i].reg_val);
	}
	dev_err(plat_data->dev, "--- TAS25XX Channel-%d RegDump done ---\n", chn);
}

/*TODO: Revisit the function as its not usually used*/
static void tas25xx_hard_reset(struct tas25xx_priv  *p_tas25xx)
{
	int i = 0;

	p_tas25xx->hw_reset(p_tas25xx);

	for (i = 0; i < p_tas25xx->ch_count; i++)
		p_tas25xx->devs[i]->mn_current_book = -1;

	if (p_tas25xx->mn_err_code)
		pr_info("%s: before reset, ErrCode=0x%x\n", __func__,
			p_tas25xx->mn_err_code);
	p_tas25xx->mn_err_code = 0;
}

int tas25xx_set_dai_fmt_for_fmt(struct tas25xx_priv *p_tas25xx, unsigned int fmt)
{
	int ret;
	int i;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		dev_info(plat_data->dev, "SND_SOC_DAIFMT_CBS_CFS\n");
		break;
	default:
		dev_err(plat_data->dev, "ASI format mask is not found\n");
		ret = -EINVAL;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(plat_data->dev, "INV format: NBNF\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_inv(p_tas25xx, i, FMT_INV_NB_NF);
			if (ret) {
				dev_err(plat_data->dev,
					"Error setting the format FMT_INV_NB_NF for ch=%d\n", i);
				break;
			}
		}
		break;

	case SND_SOC_DAIFMT_IB_NF:
		dev_info(plat_data->dev, "INV format: IBNF\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_inv(p_tas25xx, i, FMT_INV_IB_NF);
			if (ret) {
				dev_err(plat_data->dev,
					"Error setting the format FMT_INV_IB_NF for ch=%d\n", i);
				break;
			}
		}
		break;

	case SND_SOC_DAIFMT_NB_IF:
		dev_info(plat_data->dev, "INV format: NBIF\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_inv(p_tas25xx, i, FMT_INV_NB_IF);
			if (ret) {
				dev_err(plat_data->dev,
					"Error setting the format FMT_INV_NB_IF for ch=%d\n", i);
				break;
			}
		}
		break;

	case SND_SOC_DAIFMT_IB_IF:
		dev_info(plat_data->dev, "INV format: IBIF\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_inv(p_tas25xx, i, FMT_INV_IB_IF);
			if (ret) {
				dev_err(plat_data->dev,
					"Error setting the format FMT_INV_IB_IF for ch=%d\n", i);
				break;
			}
		}
		break;

	default:
		dev_err(plat_data->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	if (ret)
		return ret;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		dev_info(plat_data->dev,
			"SND_SOC_DAIFMT_I2S tdm_rx_start_slot = 1\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_mask(p_tas25xx, i, FMT_MASK_I2S);
			if (ret) {
				dev_err(plat_data->dev,
					"FMT_MASK_I2S set failed for ch=%d\n", i);
				break;
			}
		}

		break;

	case (SND_SOC_DAIFMT_DSP_A):
		dev_info(plat_data->dev,
			"SND_SOC_DAIFMT_DSP_A tdm_rx_start_slot =1\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_mask(p_tas25xx, i, FMT_MASK_DSP_A);
			if (ret) {
				dev_err(plat_data->dev,
					"FMT_MASK_DSP_A set failed for ch=%d\n", i);
				break;
			}
		}
		break;

	case (SND_SOC_DAIFMT_DSP_B):
		dev_info(plat_data->dev,
			"SND_SOC_DAIFMT_DSP_B tdm_rx_start_slot = 0\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_mask(p_tas25xx, i, FMT_MASK_DSP_B);
			if (ret) {
				dev_err(plat_data->dev,
					"FMT_MASK_DSP_B set failed for ch=%d\n", i);
				break;
			}
		}
		break;

	case (SND_SOC_DAIFMT_LEFT_J):
		dev_info(plat_data->dev,
			"SND_SOC_DAIFMT_LEFT_J tdm_rx_start_slot = 0\n");
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_set_fmt_mask(p_tas25xx, i, FMT_MASK_LEFT_J);
			if (ret) {
				dev_err(plat_data->dev,
					" set failed for ch=%d\n", i);
				break;
			}
		}
		break;

	default:
		dev_err(plat_data->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * This shall be called during the middle of the playback.
 * So all the register settings should be restored back to their original settings.
 */
static int tas25xx_reinit(struct tas25xx_priv *p_tas25xx)
{
	int i;
	int ret;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev,"%s: enter p_tas25xx->mn_fmt_mode = %d\n", __func__, p_tas25xx->mn_fmt_mode);

	ret = tas_write_init_config_params(p_tas25xx,
			p_tas25xx->ch_count);
	if (ret) {
		dev_err(plat_data->dev, "Failed to initialize, error=%d\n", ret);
		return ret;
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		ret = tas25xx_set_init_params(p_tas25xx, i);
		if (ret < 0) {
			dev_err(plat_data->dev, "%s error while initialisation for ch=%d\n",
				__func__, i);
			break;
		}
	}

	if (ret)
		goto reinit_done;

	/* set dai fmt*/
	if (p_tas25xx->mn_fmt) {
		ret = tas25xx_set_dai_fmt_for_fmt(p_tas25xx, p_tas25xx->mn_fmt);
		if (ret)
			goto reinit_done;
	}

	/* hw params */
	if (p_tas25xx->mn_fmt_mode == 2) {
		/* TDM mode */
		ret = tas25xx_set_tdm_rx_slot(p_tas25xx,
			p_tas25xx->ch_count, p_tas25xx->mn_rx_width);
		if (ret) {
			dev_err(plat_data->dev, "%s failed to set Rx slots\n", __func__);
			goto reinit_done;
		}

		ret = tas25xx_set_tdm_tx_slot(p_tas25xx,
			p_tas25xx->ch_count, p_tas25xx->mn_tx_slot_width);
		if (ret) {
			dev_err(plat_data->dev, "%s failed to set Tx slots\n", __func__);
			goto reinit_done;
		}
	} else if (p_tas25xx->mn_fmt_mode == 1) {
		/* I2S mode */
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_rx_set_bitwidth(p_tas25xx, p_tas25xx->mn_rx_width, i);
			if (ret) {
				dev_err(plat_data->dev,
					"Error=%d while setting rx bitwidth, ch=%d\n", ret, i);
				break;
			}
		}
		if (ret)
			goto reinit_done;

		ret = tas25xx_iv_vbat_slot_config(p_tas25xx, p_tas25xx->mn_tx_slot_width);
		if (ret) {
			dev_err(plat_data->dev, "Error=%d while IV vbat slot config %s\n",
				ret, __func__);
			goto reinit_done;
		}
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		ret = tas25xx_set_sample_rate(p_tas25xx, i, p_tas25xx->sample_rate);
		if (ret) {
			dev_err(plat_data->dev, "%s: Error=%d setting sample rate\n",
				__func__, ret);
			break;
		}
	}
	if (ret)
		goto reinit_done;

	ret = tas25xx_update_kcontrol_data(p_tas25xx, KCNTR_ANYTIME, 0xFFFF);
	if (ret)
		goto reinit_done;


reinit_done:
	return ret;

}

#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
void tas25xx_clear_interrupt_stats(struct tas25xx_priv *p_tas25xx)
{
	int i, j;
	struct tas25xx_intr_info *intr_info;
	struct tas25xx_interrupts *intr_data;

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		intr_data = &p_tas25xx->intr_data[i];
		for (j = 0; j < intr_data->count; j++) {
			intr_info = &intr_data->intr_info[j];
			intr_info->count = 0;
		}
	}
}

void tas25xx_log_interrupt_stats(struct tas25xx_priv *p_tas25xx)
{
	int i, j;
	static u8 log_once;
	struct tas25xx_intr_info *intr_info;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
	struct tas25xx_interrupts *intr_data;

	if (!log_once) {
		dev_info(plat_data->dev,
			"irq-bigdata: ||ch\t||name\t||count(total) ||\n");
		log_once = 1;
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		intr_data = &p_tas25xx->intr_data[i];
		for (j = 0; j < intr_data->count; j++) {
			intr_info = &intr_data->intr_info[j];
			if (intr_info->count || intr_info->count_persist)
				dev_info(plat_data->dev,
					"irq-bigdata: |%d |%s |%u(%llu) |\n", i,
					intr_info->name, intr_info->count,
					intr_info->count_persist);
		}
	}
}
#endif

/*
 * called with codec lock held
 */
int tas25xx_irq_work_func(struct tas25xx_priv *p_tas25xx)
{
	int8_t clk_intr;
	int8_t othr_intr;
	int32_t i, j;
	int32_t ret = 0;
	int32_t type = 0;
	int32_t irq_lim_crossed;
	int32_t interrupt_count;
	int32_t state;
	int32_t int_actions, power_on_required;
	uint32_t intr_detected = 0;
	int32_t reset_done = 0;
	struct tas_device *tasdev;
	struct linux_platform *plat_data;
	struct tas25xx_interrupts *intr_data;
	struct tas25xx_intr_info *intr_info;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	p_tas25xx->disable_irq(p_tas25xx);
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		ret = tas_dev_interrupt_read(p_tas25xx, i, &type);
		if (ret)
			intr_detected |= (1 << i);
	}
	p_tas25xx->enable_irq(p_tas25xx);

	if (intr_detected == 0) {
		if (is_power_up_state(p_tas25xx->m_power_state))
			for (i = 0; i < p_tas25xx->ch_count; i++)
				tas25xx_dump_regs(p_tas25xx, i);
		return ret;
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		if ((intr_detected & (1 << i)) == 0)
			continue;

		intr_data = &p_tas25xx->intr_data[i];
		interrupt_count = intr_data->count;

		irq_lim_crossed = 0;

		tasdev = p_tas25xx->devs[i];

#if IS_ENABLED(CONFIG_TISA_SYSFS_INTF)
		tas25xx_algo_bump_oc_count(0, 0);
#endif
		ret = tas_dev_interrupt_disable(p_tas25xx, i);
		if (tasdev->irq_count != 0) {
			if (time_after(jiffies, tasdev->jiffies +
				msecs_to_jiffies(TAS25XX_IRQ_DET_TIMEOUT))) {
				tasdev->jiffies = jiffies;
				tasdev->irq_count = 0;
			} else {
				tasdev->irq_count++;
				if (tasdev->irq_count > TAS25XX_IRQ_DET_CNT_LIMIT) {
					dev_err(plat_data->dev,
						"INTR: %s: ch=%d continuous interrupt detected %d\n",
						__func__, i, tasdev->irq_count);
					tas25xx_dump_regs(p_tas25xx, i);
					irq_lim_crossed = 1;
				}
			}
		} else {
			tasdev->jiffies = jiffies;
			tasdev->irq_count = 1;
		}

		if (irq_lim_crossed)
			continue;

		ret = tas_dev_interrupt_clear(p_tas25xx, i);
		if (ret)
			dev_warn(plat_data->dev,
				"%s Unable to clear interrupt, ch=%d", __func__, i);

		clk_intr = 0;
		othr_intr = 0;
		power_on_required = 1;
		int_actions = 0;
		for (j = 0; j < interrupt_count; j++) {
			intr_info = &intr_data->intr_info[j];
			if (intr_info->detected) {
				dev_info(plat_data->dev,
					"ch=%d Interrupt action for the detected interrupt %s is %d",
					i, intr_info->name, intr_info->action);
				int_actions |= intr_info->action;
				if (intr_info->action & TAS_INT_ACTION_POWER_ON)
					power_on_required = power_on_required & 1;
				else
					power_on_required = power_on_required & 0;

				if (intr_info->is_clock_based)
					clk_intr = 1;
				else
					othr_intr = 1;

				/* reset to not detected after detection */
				intr_info->detected = 0;
			}
		}

		dev_info(plat_data->dev, "ch=%d INTR force power on?(y/n):%s",
			i, power_on_required ? "y":"n");
		if (!power_on_required) {
			if (othr_intr)
				ret = tas25xx_set_power_state(p_tas25xx, TAS_POWER_SHUTDOWN, 1<<i);
			continue;
		}

		/*
		 * ||clk||oth|| action ||
		 * | 0 | 1 | power down |
		 * | 1 | 0 | do absolutely nothing |
		 * | 1 | 1 | power down |
		 */
		if (clk_intr && !othr_intr) {
			int_actions = int_actions & ~TAS_INT_ACTION_POWER_ON;
			dev_info(plat_data->dev,
				"ch=%d INTR power on ignored for [clk=%d oth=%d]",
				i, clk_intr, othr_intr);
		}

		/* order should be followed */
		if (int_actions & TAS_INT_ACTION_HW_RESET) {
			dev_info(plat_data->dev, "ch=%d Interrupt action hard reset", i);
			tas25xx_hard_reset(p_tas25xx);
			reset_done = 1;
		}

		if (int_actions & TAS_INT_ACTION_SW_RESET) {
			dev_info(plat_data->dev, "ch=%d Interrupt action software reset", i);
			ret = tas25xx_software_reset(p_tas25xx, i);
			if (ret)
				dev_err(plat_data->dev,
					"ch=%d Software reset failed error=%d", i, ret);
			reset_done = 1;
		}

		if (reset_done)
			ret = tas25xx_reinit(p_tas25xx);

		if (int_actions & TAS_INT_ACTION_POWER_ON) {
			tas25xx_check_if_powered_on(p_tas25xx, &state, i);
			if (state == 0) {
				/* interrupts are enabled during power up sequence */
				dev_info(plat_data->dev,
					"ch=%d Try powering on the device", i);
				ret = tas25xx_set_power_state(p_tas25xx,
					TAS_POWER_ACTIVE, (1<<i));
			} else {
				dev_info(plat_data->dev,
					"ch=%d Already powered on, Enable the interrupt", i);
				ret = tas_dev_interrupt_enable(p_tas25xx, i);
			}
		}
	}

	return ret;
}

int tas25xx_init_work_func(struct tas25xx_priv *p_tas25xx, struct tas_device *dev_tas25xx)
{
	int chn = 0;
	int ret = 0;
	int detected = 0;
	int type = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	chn = dev_tas25xx->channel_no;

	dev_info(plat_data->dev, "ch=%d %s\n", chn, __func__);
	ret = tas25xx_set_post_powerup(p_tas25xx, chn);
	if (ret)
		dev_err(plat_data->dev,
			"ch=%d Error in  post powerup data write.  err=%d\n",
			chn, ret);

	ret = tas25xx_update_playback_volume(p_tas25xx, chn);
	if (ret)
		dev_err(plat_data->dev,
			"ch=%d Error in update playback volume.  err=%d\n",
			chn, ret);

	/* check for interrupts during power up */
	detected = tas_dev_interrupt_read(p_tas25xx, chn, &type);
	if (detected) {
		if (type == INTERRUPT_TYPE_CLOCK_BASED) {
			/* ignore clock based interrupts which we are monitoring */
			dev_info(plat_data->dev,
				"Ignoring clock based interrupts and clear latch");
			ret = tas_dev_interrupt_clear(p_tas25xx, chn);
			if (ret)
				dev_err(plat_data->dev,
					"ch=%d Error while clearing interrup err=%d\n",
					chn, ret);
		} else {
			dev_info(plat_data->dev,
				"Non clock based interrupts detected, skip latch clear for recovery");
		}
	}

	/* enabling the interrupt here to avoid any clock errors during the bootup*/
	ret = tas_dev_interrupt_enable(p_tas25xx, chn);
	if (ret)
		dev_err(plat_data->dev,
			"ch=%d %s: Failed to enable interrupt\n", chn, __func__);

	return ret;
}

int tas25xx_dc_work_func(struct tas25xx_priv *p_tas25xx, int chn)
{
	pr_info("%s: ch %d\n", __func__, chn);
	//tas25xx_reload(p_tas25xx, chn);

	return 0;
}

int tas25xx_register_device(struct tas25xx_priv  *p_tas25xx)
{
	int i = 0;

	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s:\n", __func__);

	tas25xx_hard_reset(p_tas25xx);

	for (i = 0; i < p_tas25xx->ch_count; i++)
		p_tas25xx->devs[i]->channel_no = i;

	return 0;
}

#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
static ssize_t irq_bd_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int8_t found = 0;

	int32_t i, j;
	struct tas25xx_interrupts *intr_data;
	struct tas25xx_intr_info *intr_info;
	struct tas25xx_priv *p_tas25xx = dev_get_drvdata(dev);

	if (!p_tas25xx) {
		dev_info(dev, "dev_get_drvdata returned NULL");
		return -EINVAL;
	}

	intr_info = NULL;
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		intr_data = &p_tas25xx->intr_data[i];
		for (j = 0; j < intr_data->count; j++) {
			intr_info = &intr_data->intr_info[j];
			if (attr == intr_info->dev_attr) {
				found = 1;
				break;
			}
		}
	}

	if (found)
		return snprintf(buf, 32, "count=%u\npersist=%llu\n",
			intr_info->count, intr_info->count_persist);
	else
		return snprintf(buf, 32, "something went wrong!!!");

}

static struct attribute_group s_attribute_group = {
	.attrs = NULL,
};

static int tas_smartamp_add_irq_bd(struct tas25xx_priv *p_tas25xx)
{
	int i, j, k;
	int ret;
	int total_irqs = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
	struct device_attribute *p_irqpd;
	struct attribute **attribute_array;
	struct tas25xx_intr_info *intr_info;
	struct tas25xx_interrupts *intr_data;
	struct device *irq_dev;

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		intr_data = &p_tas25xx->intr_data[i];
		total_irqs += intr_data->count;
	}

	p_irqpd = (struct device_attribute *)kzalloc(total_irqs *
			sizeof(struct device_attribute), GFP_KERNEL);
	if (!p_irqpd)
		return -ENOMEM;

	total_irqs++;
	attribute_array = kzalloc((sizeof(struct attribute *) * total_irqs), GFP_KERNEL);

	k = 0;
	for (i = 0; i < p_tas25xx->ch_count; i++) {
		intr_data = &p_tas25xx->intr_data[i];
		for (j = 0; j < intr_data->count; j++) {
			intr_info = &intr_data->intr_info[j];
			p_irqpd[k].attr.name = intr_info->name;
			p_irqpd[k].attr.mode = 0664;
			p_irqpd[k].show = irq_bd_show;
			p_irqpd[k].store = NULL;

			intr_info->dev_attr = &p_irqpd[k];
			attribute_array[k] = &p_irqpd[k].attr;
			k++;
		}
	}
	s_attribute_group.attrs = attribute_array;

	irq_dev = device_create(p_tas25xx->class,
				NULL, 1, NULL, "irqs");
	if (IS_ERR(irq_dev)) {
		dev_err(plat_data->dev,
			"%sFailed to create irqs\n", __func__);
		ret = PTR_ERR(irq_dev);
		irq_dev = NULL;
		goto err_irqbd;
	}

	ret = sysfs_create_group(&irq_dev->kobj,
		&s_attribute_group);
	if (ret) {
		dev_err(plat_data->dev,
			"%sFailed to create sysfs group\n", __func__);
		goto err_irqbd;
	}

	p_tas25xx->irqdata.p_dev_attr = p_irqpd;
	p_tas25xx->irqdata.p_attr_arr = attribute_array;
	p_tas25xx->irqdata.irq_dev = irq_dev;

	dev_set_drvdata(irq_dev, p_tas25xx);

	dev_info(plat_data->dev, "%s ret=%d\n", __func__, ret);

	return ret;

err_irqbd:
	kfree(p_irqpd);
	kfree(attribute_array);
	return ret;
}

static void tas_smartamp_remove_irq_bd(struct tas25xx_priv *p_tas25xx)
{
	struct irq_bigdata *bd = &p_tas25xx->irqdata;

	if (bd->irq_dev)
		sysfs_remove_group(&bd->irq_dev->kobj,
			&s_attribute_group);

	kfree(bd->p_dev_attr);
	kfree(bd->p_attr_arr);

	memset(bd, 0, sizeof(struct irq_bigdata));
}
#endif

enum cmd_type_t {
	CALIB,
	TEMP,
	IV_VBAT,
	DRV_OPMODE,
};

static const char *cmd_str_arr[MAX_CMD_LIST] = {
	"calib",
	"temp",
	"iv_vbat",
	"drv_opmode",
	NULL,
};

static struct device_attribute *cmd_arr[MAX_CMD_LIST] = {
	NULL,
};

static uint8_t tas25xx_get_amb_temp(void)
{
	struct power_supply *psy;
	union power_supply_propval value = {0};

	psy = power_supply_get_by_name("battery");
	if (!psy || !psy->desc || !psy->desc->get_property) {
		pr_err("[TI-SmartPA:%s] getting ambient temp failed, using default value %d\n",
			__func__, DEFAULT_AMBIENT_TEMP);
		return DEFAULT_AMBIENT_TEMP;
	}
	psy->desc->get_property(psy, POWER_SUPPLY_PROP_TEMP, &value);

	return DIV_ROUND_CLOSEST(value.intval, 10);
}

static ssize_t cmd_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int32_t i, cmd_count, found = 0, ret = -EINVAL, temp;

	struct tas25xx_priv *p_tas25xx = dev_get_drvdata(dev);

	if (!p_tas25xx) {
		dev_info(dev,
			"%s get_drvdata returned NULL", __func__);
		return ret;
	}

	cmd_count = ARRAY_SIZE(cmd_arr);

	for (i = 0; i < cmd_count; i++) {
		if (attr == cmd_arr[i]) {
			found = 1;
			break;
		}
	}

	if (found) {
		switch (i) {
		case TEMP:
			temp = tas25xx_get_amb_temp();
			ret = snprintf(buf, 32, "%d", temp);
		break;

		case IV_VBAT:
			temp = ((p_tas25xx->curr_mn_iv_width & 0xFFFF) |
				((p_tas25xx->curr_mn_vbat & 0xFFFF) << 16));
			ret = snprintf(buf, 32, "0x%x", temp);
		break;

		case DRV_OPMODE:
			temp = tas25xx_get_drv_channel_opmode();
			ret = snprintf(buf, 32, "0x%x", temp);
		break;

		default:
			ret = snprintf(buf, 32, "unsupported cmd %d", i);
		break;
		}
	}

	return ret;
}

static ssize_t cmd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int32_t i, cmd_count, found = 0, ret = -EINVAL;
	struct tas25xx_priv *p_tas25xx = dev_get_drvdata(dev);

	if (!p_tas25xx) {
		dev_info(dev, "%s drv_data is null", __func__);
		return ret;
	}

	cmd_count = ARRAY_SIZE(cmd_arr);

	for (i = 0; i < cmd_count; i++) {
		if (attr == cmd_arr[i]) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (i == CALIB) {
			if (!strncmp(buf, "cal_init_blk", strlen("cal_init_blk")))
				tas25xx_prep_dev_for_calib(1);
			else if (!strncmp(buf, "cal_deinit_blk", strlen("cal_deinit_blk")))
				tas25xx_prep_dev_for_calib(0);
			else
				dev_info(dev,
					"%s Not supported %s, for calib", __func__, buf);
		} else {
			dev_info(dev, "%s Not supported %s", __func__, buf);
		}
	} else {
		dev_info(dev, "%s Not supported %s", __func__, buf);
	}

	return size;
}

static struct attribute_group cmd_attribute_group = {
	.attrs = NULL,
};

static int tas_smartamp_add_cmd_intf(struct tas25xx_priv *p_tas25xx)
{
	int i, cmd_count, ret;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
	struct device_attribute *p_attr_arr;
	struct attribute **attr_arry;
	struct device *cmd_dev;

	cmd_count = ARRAY_SIZE(cmd_arr);

	p_attr_arr = (struct device_attribute *)kzalloc(cmd_count *
			sizeof(struct device_attribute), GFP_KERNEL);
	if (!p_attr_arr) {
		attr_arry = NULL;
		ret = -ENOMEM;
		goto err_cmd;
	}

	attr_arry = kzalloc((sizeof(struct attribute *) *
					cmd_count), GFP_KERNEL);
	if (!attr_arry) {
		ret = -ENOMEM;
		goto err_cmd;
	}

	for (i = 0; (i < cmd_count) && cmd_str_arr[i]; i++) {
		p_attr_arr[i].attr.name = cmd_str_arr[i];
		p_attr_arr[i].attr.mode = 0664;
		p_attr_arr[i].show = cmd_show;
		p_attr_arr[i].store = cmd_store;

		cmd_arr[i] = &p_attr_arr[i];
		attr_arry[i] = &p_attr_arr[i].attr;
	}
	cmd_attribute_group.attrs = attr_arry;

	cmd_dev = device_create(p_tas25xx->class,
				NULL, 1, NULL, "cmd");
	if (IS_ERR(cmd_dev)) {
		dev_err(plat_data->dev,
			"%sFailed to create cmds\n", __func__);
		ret = PTR_ERR(cmd_dev);
		goto err_cmd;
	}

	ret = sysfs_create_group(&cmd_dev->kobj,
		&cmd_attribute_group);
	if (ret) {
		dev_err(plat_data->dev,
			"%s Failed to create sysfs group\n", __func__);
		goto err_cmd;
	}

	p_tas25xx->cmd_data.p_dev_attr = p_attr_arr;
	p_tas25xx->cmd_data.p_attr_arr = attr_arry;
	p_tas25xx->cmd_data.cmd_dev = cmd_dev;

	dev_set_drvdata(cmd_dev, p_tas25xx);

	dev_info(plat_data->dev, "%s ret=%d\n", __func__, ret);

	return ret;

err_cmd:
	kfree(p_attr_arr);
	kfree(attr_arry);
	return ret;
}

static void tas_smartamp_remove_cmd_intf(struct tas25xx_priv *p_tas25xx)
{
	struct cmd_data *cmd_data = &p_tas25xx->cmd_data;

	if (cmd_data->cmd_dev) {
		sysfs_remove_group(&cmd_data->cmd_dev->kobj,
			&cmd_attribute_group);
	}

	kfree(cmd_data->p_dev_attr);
	kfree(cmd_data->p_attr_arr);

	memset(cmd_data, 0, sizeof(struct cmd_data));
}

static int tas_smartamp_add_sysfs(struct tas25xx_priv *p_tas25xx)
{
	int ret = 0;
	struct class *class = NULL;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	class = class_create(THIS_MODULE, "tas25xx_dev");
	if (IS_ERR(class)) {
		ret = PTR_ERR(class);
		dev_err(plat_data->dev,
			"%s err class create %d\n", __func__, ret);
		class = NULL;
	}

	if (class) {
		p_tas25xx->class = class;
#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
		ret = tas_smartamp_add_irq_bd(p_tas25xx);
		if (ret)
			dev_err(plat_data->dev,
				"%s err registring irqbd %d\n", __func__, ret);
#endif
		tas_smartamp_add_cmd_intf(p_tas25xx);
	}

	return ret;
}

static void tas_smartamp_remove_sysfs(struct tas25xx_priv *p_tas25xx)
{
#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
#endif

	tas_smartamp_remove_cmd_intf(p_tas25xx);
#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
	tas_smartamp_remove_irq_bd(p_tas25xx);
	dev_info(plat_data->dev,
			"%s de-registring irqbd done\n", __func__);
#endif

	if (p_tas25xx->class) {
		device_destroy(p_tas25xx->class, 1);
		class_destroy(p_tas25xx->class);
		p_tas25xx->class = NULL;
	}
}

int tas25xx_probe(struct tas25xx_priv *p_tas25xx)
{
	int ret = -1, i = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s: initalisation writes\n", __func__);

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		ret = tas25xx_set_init_params(p_tas25xx, i);
		if (ret < 0) {
			dev_err(plat_data->dev, "%s err=%d, initialisation",
				__func__, ret);
			goto end;
		}
	}

	ret = tas25xx_create_kcontrols(p_tas25xx);
	if (ret) {
		dev_warn(plat_data->dev, "%s err=%d creating controls",
			__func__, ret);
		ret = 0;
	}

#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
	tas_smartamp_add_algo_controls(plat_data->codec, plat_data->dev,
		p_tas25xx->ch_count);
#endif
	tas_smartamp_add_sysfs(p_tas25xx);

end:
	return ret;
}

void tas25xx_remove(struct tas25xx_priv *p_tas25xx)
{
#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
	tas_smartamp_remove_algo_controls(plat_data->codec);
#endif
	/* REGBIN related */
	tas25xx_remove_binfile(p_tas25xx);
	tas_smartamp_remove_sysfs(p_tas25xx);
}

int tas25xx_set_power_state(struct tas25xx_priv *p_tas25xx,
			enum tas_power_states_t state, uint32_t ch_bitmask)
{
	int ret = 0, i = 0;
	enum tas_power_states_t cur_state;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s: state %s\n", __func__,
		(state <= TAS_POWER_SHUTDOWN) ? tas_power_states_str[state] : "INVALID");

	cur_state = p_tas25xx->m_power_state;
	p_tas25xx->m_power_state = state;

	/* supports max 4 channels */
	ch_bitmask &= tas25xx_get_drv_channel_opmode() & 0xF;

	switch (state) {
	case TAS_POWER_ACTIVE:
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			if ((ch_bitmask & (1<<i)) == 0)
				continue;

			if (cur_state != state) {
				dev_dbg(plat_data->dev,
					"ch=%d %s: clearing interrupts \n", i, __func__);

				ret = tas_dev_interrupt_clear(p_tas25xx, i);
				if (ret) {
					dev_err(plat_data->dev,
						"ch=%d %s: Error clearing interrupt\n", i, __func__);
					ret = 0;
				}
			} else {
				dev_dbg(plat_data->dev,
					"ch=%d %s: skipping clearing interrupts \n", i, __func__);
			}

			ret = tas25xx_set_pre_powerup(p_tas25xx, i);
			if (ret) {
				dev_err(plat_data->dev, "ch=%d %s setting power state failed, err=%d\n",
					i, __func__, ret);
			} else {
				ret = tas25xx_update_kcontrol_data(p_tas25xx, KCNTR_PRE_POWERUP, (1 << i));
				p_tas25xx->schedule_init_work(p_tas25xx, i);
			}
		}
		break;

	case TAS_POWER_MUTE:
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			if ((ch_bitmask & (1<<i)) == 0)
				continue;
			tas25xx_set_power_mute(p_tas25xx, i);
		}
		break;

	case TAS_POWER_SHUTDOWN:
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			if ((ch_bitmask & (1<<i)) == 0)
				continue;

			/* device interrupt disable */
			tas_dev_interrupt_disable(p_tas25xx, i);
			p_tas25xx->cancel_init_work(p_tas25xx, i);
			ret = tas25xx_set_pre_powerdown(p_tas25xx, i);
			ret |= tas25xx_set_post_powerdown(p_tas25xx, i);
		}
		break;

	default:
		ret = -EINVAL;
		dev_err(plat_data->dev, "wrong power state setting %d\n",
				state);
	}

	return ret;
}

static int find_fmt_match(struct tas25xx_priv *p_tas25xx, char *fmt, uint8_t **in_out_buf)
{
	int ret, i;
	uint32_t blk_count, sublk_sz, fmt_len;
	uint8_t *buf;
	//struct linux_platform *plat_data;

	ret = -EINVAL;
	buf = *in_out_buf;
	//plat_data =	(struct linux_platform *) p_tas25xx->platform_data;

	//size = *((uint32_t *)buf);
	buf += sizeof(uint32_t);
	blk_count = *((uint32_t *)buf);
	buf += sizeof(uint32_t);

	if (!fmt) {
		*in_out_buf = NULL;
		return ret;
	}

	fmt_len = strlen(fmt);
	for (i = 0; i < blk_count; i++) {
		if (memcmp(fmt, buf, fmt_len) == 0) {
			/* block found */
			buf += fmt_len;
			break;
		} else {
			/* move to next block */
			buf += fmt_len;
			sublk_sz = *((uint32_t *)buf);
			buf += sizeof(uint32_t) + sublk_sz;
		}
	}

	if (i < blk_count) {
		*in_out_buf = buf;
		ret = i;
	} else {
		*in_out_buf = NULL;
	}

	return ret;
}

static int get_next_possible_iv_width(int current_iv_width)
{
	int ret;

	switch (current_iv_width) {
	case 16:
		ret = 12;
		break;

	case 12:
		ret = 8;
		break;

	case 8:
	default:
		ret = -1;
		break;
	}

	return ret;
}

int tas25xx_iv_vbat_slot_config(struct tas25xx_priv *p_tas25xx,
	int mn_slot_width)
{
	int i;
	int ret = -EINVAL;
	int iv_width, prev_iv_width, vbat_on;
	int near_match_found = 0;
	char tx_fmt[32];
	struct linux_platform *plat_data;

	/* treating 24bit and 32bit as same */
	if (mn_slot_width == 32)
		mn_slot_width = 24;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	scnprintf(tx_fmt, 15, "%2d_%s_%02d_%02d_%1d", mn_slot_width,
		p_tas25xx->mn_fmt_mode == 2 ? "TDM" : "I2S", p_tas25xx->ch_count,
		p_tas25xx->mn_iv_width, p_tas25xx->mn_vbat);

	dev_info(plat_data->dev, "finding exact match for %s", tx_fmt);

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		uint8_t *data = p_tas25xx->block_op_data[i].tx_fmt_data;

		iv_width = p_tas25xx->mn_iv_width;
		vbat_on = p_tas25xx->mn_vbat;

		/* Try to find exact match */
		if ((i > 0) && (near_match_found)) {
			dev_info(plat_data->dev, "same format is being used %s, ch=%d", tx_fmt, i);
		} else {
			scnprintf(tx_fmt, 15, "%2d_%s_%02d_%02d_%1d", mn_slot_width,
				p_tas25xx->mn_fmt_mode == 2 ? "TDM" : "I2S", p_tas25xx->ch_count,
				p_tas25xx->mn_iv_width, p_tas25xx->mn_vbat);
		}

		if (find_fmt_match(p_tas25xx, tx_fmt, &data) >= 0) {
			/* match found */
			dev_info(plat_data->dev, "exact match found for %s, ch=%d", tx_fmt, i);
		} else {
			/* try near possible option */
			dev_err(plat_data->dev, "exact match was not found for %s, trying to find near match", tx_fmt);
			/*1. Try with reduced bit width - 16 -> 12 -> 8 */
			while (1) {
				prev_iv_width = iv_width;
				iv_width = get_next_possible_iv_width(prev_iv_width);
				if (iv_width < 0) {
					if (vbat_on) {
						/*2. reset iv width and check with vbat off */
						iv_width = p_tas25xx->mn_iv_width;
						vbat_on = 0;
					} else {
						data = NULL;
						break;
					}
				}
				scnprintf(tx_fmt, 15, "%2d_%s_%02d_%02d_%1d", mn_slot_width,
					p_tas25xx->mn_fmt_mode == 2 ? "TDM" : "I2S", p_tas25xx->ch_count,
					iv_width, vbat_on);
				dev_info(plat_data->dev, "finding near match with %s", tx_fmt);
				/* reset data for fresh search with new string */
				data = p_tas25xx->block_op_data[i].tx_fmt_data;
				if (find_fmt_match(p_tas25xx, tx_fmt, &data) >= 0) {
					dev_info(plat_data->dev, "near match found: %s for ch=%d", tx_fmt, i);
					near_match_found |= (1 << i);
					break;
				}
			} /* <while loop> */
		} /* if-else */

		if (data) {
			dev_info(plat_data->dev, "possible/exact match found %s for iv=%d, vbat=%d",
				tx_fmt, p_tas25xx->mn_iv_width, p_tas25xx->mn_vbat);
			ret = tas25xx_process_block(p_tas25xx, (char *)data, i);
			/* error setting iv width */
			if (ret) {
				dev_info(plat_data->dev, "process block error for %s for iv=%d, vbat=%d",
					tx_fmt, p_tas25xx->mn_iv_width, p_tas25xx->mn_vbat);
			}
		} else {
			dev_err(plat_data->dev, "no near match found iv=%d, vbat=%d",
				p_tas25xx->mn_iv_width, p_tas25xx->mn_vbat);
			ret = -EINVAL;
			break;
		}
	} /* <for loop> */

	if (ret == 0) {
		p_tas25xx->curr_mn_iv_width = iv_width;
		p_tas25xx->curr_mn_vbat = vbat_on;
		p_tas25xx->mn_tx_slot_width = mn_slot_width;
	}

	return ret;
}

/* tas25xx_set_bitwidth function is redesigned to accomodate change in
 * tas25xx_iv_vbat_slot_config()
 */
int tas25xx_set_bitwidth(struct tas25xx_priv *p_tas25xx,
	int bitwidth, int stream)
{
	int i;
	int ret = -EINVAL;
	struct linux_platform *plat_data;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s: bitwidth %d stream %d\n", __func__, bitwidth, stream);

	if (stream == TAS25XX_STREAM_PLAYBACK) {
		for (i = 0; i < p_tas25xx->ch_count; i++) {
			ret = tas25xx_rx_set_bitwidth(p_tas25xx, bitwidth, i);
			if (ret) {
				dev_err(plat_data->dev, "Error =%d while setting bitwidth, ch=%d,",
					ret, i);
			}
		}
	/*stream == TAS25XX_STREAM_CAPTURE*/
	} else {
		ret = tas25xx_iv_vbat_slot_config(p_tas25xx, bitwidth);
		if (ret)
			dev_err(plat_data->dev, "Error =%d with %s", ret, __func__);
	}

	return ret;
}

/* tas25xx_set_tdm_rx_slot function is redesigned to accomodate change in
 * tas25xx_iv_vbat_slot_config()
 */
int tas25xx_set_tdm_rx_slot(struct tas25xx_priv *p_tas25xx,
	int slots, int slot_width)
{
	int i;
	int ret = -1;
	struct linux_platform *plat_data;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s: slots=%d, slot_width=%d",
		__func__, slots, slot_width);

	if (((p_tas25xx->ch_count == 1) && (slots < 1)) ||
		((p_tas25xx->ch_count == 2) && (slots < 2))) {
		dev_err(plat_data->dev, "Invalid Slots %d\n", slots);
		return ret;
	}
	p_tas25xx->mn_slots = slots;

	if ((slot_width != 16) &&
		(slot_width != 24) &&
		(slot_width != 32)) {
		dev_err(plat_data->dev, "Unsupported slot width %d\n", slot_width);
		return ret;
	}

	for (i = 0; i < p_tas25xx->ch_count; i++) {
		ret = tas25xx_rx_set_bitwidth(p_tas25xx, slot_width, i);
		if (ret) {
			dev_err(plat_data->dev, "Error =%d while setting bitwidth, ch=%d,",
				ret, i);
		}
	}

	return ret;
}

/* tas25xx_set_tdm_tx_slot function is redesigned to accomodate change in
 * tas25xx_iv_vbat_slot_config()
 */
int tas25xx_set_tdm_tx_slot(struct tas25xx_priv *p_tas25xx,
	int slots, int slot_width)
{
	int ret = -1;

	if ((slot_width != 16) &&
		(slot_width != 24) &&
		(slot_width != 32)) {
		pr_err("Unsupported slot width %d\n", slot_width);
		return ret;
	}

	if (((p_tas25xx->ch_count == 1) && (slots < 2)) ||
		((p_tas25xx->ch_count == 2) && (slots < 4))) {
		pr_err("Invalid Slots %d\n", slots);
		return ret;
	}
	p_tas25xx->mn_slots = slots;

	ret = tas25xx_iv_vbat_slot_config(p_tas25xx, slot_width);

	return ret;
}
