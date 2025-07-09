/*
 * ALSA SoC Texas Instruments TAS25XX High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2022 Texas Instruments, Inc.
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
#include "inc/tas25xx.h"
#include "inc/tas25xx-device.h"
#include "inc/tas25xx-regbin-parser.h"
#include "inc/tas25xx-regmap.h"

#define TAS25XX_MDELAY 0xFFFFFFFE
#define TAS25XX_MSLEEP 0xFFFFFFFD
#define TAS25XX_IVSENSER_ENABLE  1
#define TAS25XX_IVSENSER_DISABLE 0

#define STR_RXBITS_SZ 5
#define STR_16BIT "16BIT"
#define STR_24BIT "24BIT"
#define	STR_32BIT "32BIT"

#ifndef UINT64_MAX
#define UINT64_MAX	((u64)(~((u64)0)))
#endif

#ifndef UINT32_MAX
#define UINT32_MAX	((u32)(~((u32)0)))
#endif

int tas25xx_rx_set_bitwidth(struct tas25xx_priv *p_tas25xx,
	int bitwidth, int ch)
{
	int ret;
	int32_t i;
	uint32_t blk_count, sublk_sz;
	uint8_t *rx_data = p_tas25xx->block_op_data[ch].rx_fmt_data;
	uint8_t *fmt;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	ret = 0;
	rx_data += sizeof(uint32_t);
	blk_count = *((uint32_t *)rx_data);
	rx_data += sizeof(uint32_t);

	switch (bitwidth) {
	case 16:
		fmt = STR_16BIT;
		break;

	case 24:
		fmt = STR_24BIT;
		break;

	case 32:
		fmt = STR_32BIT;
		break;

	default:
		fmt = NULL;
		dev_info(plat_data->dev,
			"Not supported params bitwidth %d",
					bitwidth);
		break;
	}

	if (!fmt)
		return -EINVAL;

	for (i = 0; i < blk_count; i++) {
		if (memcmp(fmt, rx_data, STR_RXBITS_SZ) == 0) {
			rx_data += STR_RXBITS_SZ;
			break;
		} else {
			rx_data += STR_RXBITS_SZ;
			sublk_sz = *((uint32_t *)rx_data);
			rx_data += sizeof(uint32_t);
			rx_data += sublk_sz;
		}
	}

	if (i < blk_count)
		ret = tas25xx_process_block(p_tas25xx, rx_data, ch);
	else
		ret = -EINVAL;

	if (ret == 0)
		p_tas25xx->mn_rx_width = bitwidth;

	return ret;
}

int tas_dev_interrupt_clear(struct tas25xx_priv *p_tas25xx, int chn)
{
	if (p_tas25xx->intr_data[chn].buf_intr_clear)
		return tas25xx_process_block(p_tas25xx,
			p_tas25xx->intr_data[chn].buf_intr_clear, chn);

	return 0;
}

int tas_dev_interrupt_enable(struct tas25xx_priv *p_tas25xx, int chn)
{
	p_tas25xx->devs[chn]->irq_count = 0;

	if (p_tas25xx->intr_data[chn].buf_intr_enable)
		return tas25xx_process_block(p_tas25xx,
			p_tas25xx->intr_data[chn].buf_intr_enable, chn);

	return 0;
}

int tas_dev_interrupt_disable(struct tas25xx_priv *p_tas25xx, int chn)
{
	if (p_tas25xx->intr_data[chn].buf_intr_disable)
		return tas25xx_process_block(p_tas25xx,
			p_tas25xx->intr_data[chn].buf_intr_disable, chn);

	return 0;
}

int tas_dev_interrupt_read(struct tas25xx_priv *p_tas25xx, int chn, int *type)
{
	int32_t ret = 0;
	int32_t i;
	int32_t reg = -1;
	int32_t intr_detected = 0;
	uint32_t value = 0;
	int32_t powered_up = 0;

	struct tas25xx_intr_info *intr_info;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;
	struct tas25xx_interrupts *intr_data = &p_tas25xx->intr_data[chn];

	powered_up = is_power_up_state(p_tas25xx->m_power_state);

	for (i = 0; i < intr_data->count; i++) {
		intr_info = &intr_data->intr_info[i];
		if (!powered_up && intr_info->is_clock_based) {
			/* ignore clock based interrupt during power off state */
			dev_dbg(plat_data->dev,
				"INTR: not checking for %s, reason: not active state",
				intr_info->name);
			continue;
		}

		if ((reg != intr_info->reg) || ret) {
			reg = intr_info->reg;
			ret = p_tas25xx->read(p_tas25xx, chn, reg, &value);
			if (!ret)
				dev_info(plat_data->dev,
					"INTR: ch=%d reg=0x%2x(%d), value=0x%2x(%d)",
					chn, reg, reg, value, value);
		} else {
			dev_dbg(plat_data->dev, "INTR: skipping reading reg = %d",
				intr_info->reg);
		}
		if (ret) {
			dev_err(plat_data->dev,
				"INTR: Error reading the interrupt reg=%d, err=%d", reg, ret);
		} else {
			if (value & intr_info->mask) {
				if (powered_up && intr_info->is_clock_based)
					*type |= INTERRUPT_TYPE_CLOCK_BASED;
				else
					*type |= INTERRUPT_TYPE_NON_CLOCK_BASED;
				dev_info(plat_data->dev, "INTR: Detected, ch=%d, intr=%s",
					chn, intr_info->name);
				intr_info->detected = 1;
				if (intr_info->count < UINT32_MAX)
					intr_info->count++;
				if (intr_info->count_persist < UINT64_MAX)
					intr_info->count_persist++;
				intr_detected |= 1;
			}
		}
	}

	return intr_detected;
}

int tas25xx_set_power_mute(struct tas25xx_priv *p_tas25xx, int ch)
{
	return tas25xx_process_block(p_tas25xx,
		p_tas25xx->block_op_data[ch].mute, ch);
}

int tas25xx_software_reset(struct tas25xx_priv *p_tas25xx, int ch)
{
	int ret;

	ret = tas25xx_process_block(p_tas25xx,
		p_tas25xx->block_op_data[ch].sw_reset, ch);

	p_tas25xx->devs[ch]->mn_current_book = -1;

	msleep(20);

	return ret;
}
