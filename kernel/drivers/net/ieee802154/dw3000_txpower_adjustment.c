/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#include <linux/types.h>
#include "dw3000_trc.h"
#include "dw3000_txpower_adjustment.h"

/* Size of Tx power compensation look-up table */
#define LUT_COMP_SIZE 64

/* Margin for tx power adjustment in 0.1dB steps. */
#define TXPOWER_ADJUSTMENT_MARGIN 5

/* TxPower difference between coarse gain n and coarse gain n+1 in 0.1dB Step.
 * Same for CH5 and CH9 */
#define COARSE_0_TO_1 32
#define COARSE_1_TO_2 13
#define COARSE_2_TO_3 5
#define NUM_COARSE_GAIN 3

/* LookUpTable  : 1000us - 200us */
#define LUT_1000_200_US_NUM 33
#define LUT_1000_200_US_STEP 25
#define LUT_1000_200_US_MIN 200
#define LUT_1000_200_US_MIN_BST 0

/* LookUpTable  : 200us - 70us */
#define LUT_200_70_US_NUM 14
#define LUT_200_70_US_STEP 10
#define LUT_200_70_US_MIN 70
#define LUT_200_70_US_MAX_BST 113

/* The reference duration for a frame is 1000us. Longer frame will
 * have 0dB boost. */
#define FRAME_DURATION_REF 1000

#define MAX_BOOST_CH5 354
#define MAX_BOOST_CH9 305
#define TX_POWER_COARSE_GAIN_MASK 0x03
#define TX_POWER_FINE_GAIN_MASK 0x3F

/* Reference look-up table to calculate Txpower Dial Back depending on
 * frame duration. Each new entry in the table is relative to the previous one.
 * Using two different tables a per logarithmic calculation:
 * - 1000us to 200us range (index unit of 25us)
 * - 200us to 70us (index unit of 10us)
 * The table values are in steps 0f 0.1dB
 * This allows having a maximum granularity of 0.5dB between two frame
 * duration, which is in the magnitude of the DW3XXX TxOutput setting.
 */
static const u8
	txpower_boost_per_frame_duration_1000_200_us[LUT_1000_200_US_NUM] = {
		0, /* 1000us */
		1, /* 975us  -> 1*0.1dB boost between 975us and 1000us frames */
		2, /* 950us  -> 2*0.1dB boost between 950us and 1000us frames */
		3, /* 925us */
		4, /* 900us */
		5, /* 875us */
		6, /* 850us */
		7, /* 825us */
		8, /* 800us */
		9, /* 775us */
		10, /* 750us */
		11, /* 725us */
		13, /* 700us */
		15, /* 675us */
		17, /* 650us */
		19, /* 625us */
		21, /* 600us */
		23, /* 575us */
		25, /* 550us */
		27, /* 525us */
		29, /* 500us */
		31, /* 475us */
		33, /* 450us */
		35, /* 425us */
		38, /* 400us */
		41, /* 375us */
		44, /* 350us */
		47, /* 325us */
		50, /* 300us */
		54, /* 275us */
		58, /* 250us */
		63, /* 225us */
		68 /* 200us */
	};

static const u8 txpower_boost_per_frame_duration_200_70_us[LUT_200_70_US_NUM] = {
	68, /* 200us  -> 68*0.1dB boost between 200us frame and 1000us frame. */
	70, /* 190us  -> 70*0.1dB boost between 190us and 1000us frame */
	72, /* 180us */
	74, /* 170us */
	77, /* 160us */
	80, /* 150us */
	83, /* 140us */
	86, /* 130us */
	89, /* 120us */
	93, /* 110us */
	97, /* 100us */
	102, /* 90us */
	107, /* 80us */
	113 /* 70us */
};

/* TxPower difference between the coarse gain setting (i+1) and (i)
 * The step is in unit of 0.1dB
 */
static const u8 lut_coarse_gain[NUM_COARSE_GAIN] = {
	COARSE_0_TO_1,
	COARSE_1_TO_2,
	COARSE_2_TO_3,
};

/* TxPower difference between the fine gain setting (i+1) and (i)
 * The step is in unit of 0.1dB
 */
static const u8 fine_gain_lut_chan5[LUT_COMP_SIZE] = {
	0, /*  Fine gain setting ( 0 ) Minimum Value */
	32, /*  Fine gain setting ( 1  -  0  ) */
	29, /*  Fine gain setting ( 2  -  1  ) */
	28, /*  Fine gain setting ( 3  -  2  ) */
	20, /*  Fine gain setting ( 4  -  3  ) */
	18, /*  Fine gain setting ( 5  -  4  ) */
	12, /*  Fine gain setting ( 6  -  5  ) */
	13, /*  Fine gain setting ( 7  -  6  ) */
	10, /*  Fine gain setting ( 8  -  7  ) */
	10, /*  Fine gain setting ( 9  -  8  ) */
	7, /*  Fine gain setting ( 10 -  9  ) */
	8, /*  Fine gain setting ( 11 -  10 ) */
	6, /*  Fine gain setting ( 12 -  11 ) */
	7, /*  Fine gain setting ( 13 -  12 ) */
	5, /*  Fine gain setting ( 14 -  13 ) */
	6, /*  Fine gain setting ( 15 -  14 ) */
	5, /*  Fine gain setting ( 16 -  15 ) */
	5, /*  Fine gain setting ( 17 -  16 ) */
	4, /*  Fine gain setting ( 18 -  17 ) */
	4, /*  Fine gain setting ( 19 -  18 ) */
	4, /*  Fine gain setting ( 20 -  19 ) */
	4, /*  Fine gain setting ( 21 -  20 ) */
	3, /*  Fine gain setting ( 22 -  21 ) */
	3, /*  Fine gain setting ( 23 -  22 ) */
	3, /*  Fine gain setting ( 24 -  23 ) */
	3, /*  Fine gain setting ( 25 -  24 ) */
	2, /*  Fine gain setting ( 26 -  25 ) */
	3, /*  Fine gain setting ( 27 -  26 ) */
	2, /*  Fine gain setting ( 28 -  27 ) */
	3, /*  Fine gain setting ( 29 -  28 ) */
	2, /*  Fine gain setting ( 30 -  29 ) */
	3, /*  Fine gain setting ( 31 -  30 ) */
	3, /*  Fine gain setting ( 32 -  31 ) */
	2, /*  Fine gain setting ( 33 -  32 ) */
	2, /*  Fine gain setting ( 34 -  33 ) */
	2, /*  Fine gain setting ( 35 -  34 ) */
	1, /*  Fine gain setting ( 36 -  35 ) */
	2, /*  Fine gain setting ( 37 -  36 ) */
	1, /*  Fine gain setting ( 38 -  37 ) */
	2, /*  Fine gain setting ( 39 -  38 ) */
	1, /*  Fine gain setting ( 40 -  39 ) */
	2, /*  Fine gain setting ( 41 -  40 ) */
	1, /*  Fine gain setting ( 42 -  41 ) */
	1, /*  Fine gain setting ( 43 -  42 ) */
	1, /*  Fine gain setting ( 44 -  43 ) */
	1, /*  Fine gain setting ( 45 -  44 ) */
	1, /*  Fine gain setting ( 46 -  45 ) */
	1, /*  Fine gain setting ( 47 -  46 ) */
	1, /*  Fine gain setting ( 48 -  47 ) */
	1, /*  Fine gain setting ( 49 -  48 ) */
	1, /*  Fine gain setting ( 50 -  49 ) */
	1, /*  Fine gain setting ( 51 -  50 ) */
	1, /*  Fine gain setting ( 52 -  51 ) */
	1, /*  Fine gain setting ( 53 -  52 ) */
	1, /*  Fine gain setting ( 54 -  53 ) */
	1, /*  Fine gain setting ( 55 -  54 ) */
	1, /*  Fine gain setting ( 56 -  55 ) */
	1, /*  Fine gain setting ( 57 -  56 ) */
	1, /*  Fine gain setting ( 58 -  57 ) */
	1, /*  Fine gain setting ( 59 -  58 ) */
	1, /*  Fine gain setting ( 60 -  59 ) */
	1, /*  Fine gain setting ( 61 -  60 ) */
	1, /*  Fine gain setting ( 62 -  61 ) */
	1 /*  Fine gain setting ( 63 -  62 ) */
};

static const u8 fine_gain_lut_chan9[LUT_COMP_SIZE] = {
	0, /*  Fine gain setting ( 0 ) Minimum Value */
	11, /*  Fine gain setting ( 1  -  0  ) */
	14, /*  Fine gain setting ( 2  -  1  ) */
	18, /*  Fine gain setting ( 3  -  2  ) */
	15, /*  Fine gain setting ( 4  -  3  ) */
	15, /*  Fine gain setting ( 5  -  4  ) */
	10, /*  Fine gain setting ( 6  -  5  ) */
	12, /*  Fine gain setting ( 7  -  6  ) */
	9, /*  Fine gain setting ( 8  -  7  ) */
	9, /*  Fine gain setting ( 9  -  8  ) */
	7, /*  Fine gain setting ( 10 -  9  ) */
	8, /*  Fine gain setting ( 11 -  10 ) */
	6, /*  Fine gain setting ( 12 -  11 ) */
	7, /*  Fine gain setting ( 13 -  12 ) */
	5, /*  Fine gain setting ( 14 -  13 ) */
	6, /*  Fine gain setting ( 15 -  14 ) */
	5, /*  Fine gain setting ( 16 -  15 ) */
	5, /*  Fine gain setting ( 17 -  16 ) */
	4, /*  Fine gain setting ( 18 -  17 ) */
	5, /*  Fine gain setting ( 19 -  18 ) */
	4, /*  Fine gain setting ( 20 -  19 ) */
	4, /*  Fine gain setting ( 21 -  20 ) */
	3, /*  Fine gain setting ( 22 -  21 ) */
	4, /*  Fine gain setting ( 23 -  22 ) */
	3, /*  Fine gain setting ( 24 -  23 ) */
	3, /*  Fine gain setting ( 25 -  24 ) */
	3, /*  Fine gain setting ( 26 -  25 ) */
	3, /*  Fine gain setting ( 27 -  26 ) */
	3, /*  Fine gain setting ( 28 -  27 ) */
	3, /*  Fine gain setting ( 29 -  28 ) */
	2, /*  Fine gain setting ( 30 -  29 ) */
	3, /*  Fine gain setting ( 31 -  30 ) */
	3, /*  Fine gain setting ( 32 -  31 ) */
	2, /*  Fine gain setting ( 33 -  32 ) */
	2, /*  Fine gain setting ( 34 -  33 ) */
	2, /*  Fine gain setting ( 35 -  34 ) */
	2, /*  Fine gain setting ( 36 -  35 ) */
	2, /*  Fine gain setting ( 37 -  36 ) */
	1, /*  Fine gain setting ( 38 -  37 ) */
	2, /*  Fine gain setting ( 39 -  38 ) */
	2, /*  Fine gain setting ( 40 -  39 ) */
	2, /*  Fine gain setting ( 41 -  40 ) */
	2, /*  Fine gain setting ( 42 -  41 ) */
	2, /*  Fine gain setting ( 43 -  42 ) */
	1, /*  Fine gain setting ( 44 -  43 ) */
	2, /*  Fine gain setting ( 45 -  44 ) */
	1, /*  Fine gain setting ( 46 -  45 ) */
	2, /*  Fine gain setting ( 47 -  46 ) */
	1, /*  Fine gain setting ( 48 -  47 ) */
	1, /*  Fine gain setting ( 49 -  48 ) */
	1, /*  Fine gain setting ( 50 -  49 ) */
	2, /*  Fine gain setting ( 51 -  50 ) */
	1, /*  Fine gain setting ( 52 -  51 ) */
	1, /*  Fine gain setting ( 53 -  52 ) */
	1, /*  Fine gain setting ( 54 -  53 ) */
	1, /*  Fine gain setting ( 55 -  54 ) */
	1, /*  Fine gain setting ( 56 -  55 ) */
	1, /*  Fine gain setting ( 57 -  56 ) */
	1, /*  Fine gain setting ( 58 -  57 ) */
	1, /*  Fine gain setting ( 59 -  58 ) */
	0, /*  Fine gain setting ( 60 -  59 ) */
	1, /*  Fine gain setting ( 61 -  60 ) */
	1, /*  Fine gain setting ( 62 -  61 ) */
	1 /*  Fine gain setting ( 63 -  62 ) */
};

/* Calculate the power_boost for a frame_duration_us relative to 1ms */
static u8 calculate_power_boost(u16 frame_duration_us)
{
	const u8 *lut = NULL;
	u16 lut_i;
	u16 lut_num;
	u16 lut_min;
	u16 lut_step;
	u16 limit;

	/* Calculating the LUT index corresponding to the frameduration */
	if (frame_duration_us >= FRAME_DURATION_REF) {
		return LUT_1000_200_US_MIN_BST;
	} else if (frame_duration_us < LUT_200_70_US_MIN) {
		return LUT_200_70_US_MAX_BST;
	} else if (frame_duration_us > LUT_1000_200_US_MIN) {
		lut_num = LUT_1000_200_US_NUM;
		lut_min = LUT_1000_200_US_MIN;
		lut_step = LUT_1000_200_US_STEP;
		lut = txpower_boost_per_frame_duration_1000_200_us;
	} else {
		lut_num = LUT_200_70_US_NUM;
		lut_min = LUT_200_70_US_MIN;
		lut_step = LUT_200_70_US_STEP;
		lut = txpower_boost_per_frame_duration_200_70_us;
	}

	lut_i = (lut_num - (frame_duration_us - lut_min) / lut_step);
	limit = (lut_num - lut_i) * lut_step + lut_min;

	/* Selecting the index that gives the closest LUT */
	if (abs(frame_duration_us - limit) > lut_step / 2) {
		lut_i--;
	}

	return lut[lut_i - 1];
}

/**
 * adjust_tx_power() - actual TxPower setting calculation
 * @frame_duration_us: the frame (headers, payload, FCS) duration
 * @ref_tx_power: the power setting corresponding to a frame of 1ms (0dB)
 * @channel: the current RF channel used for transmission of UWB frames
 * @th_boost: pointer to store the theorical boost to be applied
 * @applied_boost: pointer to store the calculated, actually applied boost
 *
 * The reference TxPower setting should correspond to a 1ms frame (or 0dB)
 * boost. The boost to be applied should be provided in unit of 0.1dB boost.
 *
 * Return: the adjusted_tx_power setting that can be used to configure the
 * chip transmission level
 */
static u32 adjust_tx_power(u16 frame_duration_us, u32 ref_tx_power, u8 channel,
			   u16 *th_boost, u16 *applied_boost)
{
	u32 adjusted_tx_power;
	u16 target_boost = 0;
	u16 base_target_boost = 0;
	u16 current_boost = 0;
	u16 best_boost_abs = 0;
	u16 best_boost = 0;
	u16 upper_limit = 0;
	u16 lower_limit = 0;

	const u8 *lut = NULL;
	uint8_t ref_tx_power_byte[4]; /* txpwr of each segment (UM 8.2.2.20) */
	uint8_t adj_tx_power_byte[4];
	uint8_t adj_tx_power_boost[4];
	u8 best_index;
	u8 best_coarse_gain;
	u8 ref_coarse_gain;
	u8 ref_fine_gain;
	bool within_margin_flag;
	bool reached_max_fine_gain_flag;
	bool shortcut_optim_flag;
	u8 unlock;
	u8 i, j;
	int k;
	int ret = 0;

	base_target_boost = calculate_power_boost(frame_duration_us);
	if (th_boost) {
		*th_boost = base_target_boost;
	}
	switch (channel) {
	case 5:
		lut = fine_gain_lut_chan5;
		if (base_target_boost > MAX_BOOST_CH5)
			base_target_boost = MAX_BOOST_CH5;
		break;
	default:
		lut = fine_gain_lut_chan9;
		if (base_target_boost > MAX_BOOST_CH9)
			base_target_boost = MAX_BOOST_CH9;
		break;
	}

	for (k = 0; k < 4; k++) {
		target_boost = base_target_boost;
		current_boost = 0;
		best_boost_abs = 0;
		best_boost = 0;
		best_index = 0;
		best_coarse_gain = 0;
		within_margin_flag = false;
		reached_max_fine_gain_flag = false;
		shortcut_optim_flag = false;
		unlock = 0;
		i = 0;
		ref_tx_power_byte[k] = (u8)(ref_tx_power >> (k << 3));
		ref_coarse_gain = ((u8)ref_tx_power_byte[k]) &
				  TX_POWER_COARSE_GAIN_MASK;
		ref_fine_gain = ((u8)ref_tx_power_byte[k] >> 2) &
				TX_POWER_FINE_GAIN_MASK;
		adj_tx_power_boost[k] = 0;
		i = ref_fine_gain;

		/* Avoid re-doing the same math four times */
		for (j = 0; !shortcut_optim_flag && (j < k); j++) {
			if (ref_tx_power_byte[k] == ref_tx_power_byte[j]) {
				adj_tx_power_byte[k] = adj_tx_power_byte[j];
				shortcut_optim_flag = true;
			}
		}
		if (shortcut_optim_flag)
			continue;

		/* PHR power must be 6dB lower than PSDU */
		if (k == 1) {
			uint8_t psdu_boost_err =
				(target_boost > adj_tx_power_boost[0] ?
					 target_boost - adj_tx_power_boost[0] :
					 0);
			if (psdu_boost_err < 0)
				psdu_boost_err = 0;
			target_boost -= psdu_boost_err;
		}

		upper_limit = target_boost + TXPOWER_ADJUSTMENT_MARGIN;
		lower_limit =
			(target_boost > TXPOWER_ADJUSTMENT_MARGIN ?
				 target_boost - TXPOWER_ADJUSTMENT_MARGIN :
				 0);
		best_boost_abs = TXPOWER_ADJUSTMENT_MARGIN;

		if (target_boost < TXPOWER_ADJUSTMENT_MARGIN &&
		    target_boost < lut[i + 1] - TXPOWER_ADJUSTMENT_MARGIN) {
			if (k == 0 && applied_boost)
				*applied_boost = 0;
			adj_tx_power_byte[k] = ref_tx_power_byte[k];
			continue;
		}

		/* Increase coarse setting if the required boost is greater than the
		 * TxPower gain using the increased coarse setting.
		 * NB : Coarse gain = 0x3 should not be used in CHAN9 */
		while (ref_coarse_gain < 0x2) {
			if (lut_coarse_gain[ref_coarse_gain] <
			    target_boost - current_boost) {
				current_boost +=
					lut_coarse_gain[ref_coarse_gain];
				ref_coarse_gain++;
			} else {
				break;
			}
		}

		/* Increase current_boost until reaching value closest to target_boost */
		while (current_boost != target_boost) {
			unlock++;
			/* Ensure loop does not got "locked" */
			if (unlock > 2 * LUT_COMP_SIZE) {
				ret = -EFAULT;
				break;
			}

			if (current_boost > lower_limit &&
			    current_boost < upper_limit) {
				if (abs((s16)(target_boost - current_boost)) <=
				    best_boost_abs) {
					best_boost_abs = abs((s16)target_boost - current_boost);
					best_boost = current_boost;
					best_index = i;
					best_coarse_gain = ref_coarse_gain;
					within_margin_flag = true;
				} else if (within_margin_flag) {
					i = best_index;
					ref_coarse_gain = best_coarse_gain;
					break;
				}
			} else if (within_margin_flag) {
				current_boost -= lut[i];
				i = best_index;
				break;
			}

			/* Corner case: when fine gain setting is very low, it can happened that
			 * current boost is already larger than target_boost but not within margin.
			 * Then, just return current solution. */
			if (current_boost >= upper_limit &&
			    !reached_max_fine_gain_flag) {
				break;
			}

			/* Search for max fine gain value */
			if (i == LUT_COMP_SIZE - 1) {
				reached_max_fine_gain_flag = true;

				/* return previously found solution */
				if (within_margin_flag) {
					i = best_index;
					ref_coarse_gain = best_coarse_gain;
					current_boost = best_boost;
					break;
				}

				if ((ref_coarse_gain == 0x3) ||
				    (ref_coarse_gain == 0x2 && channel == 9)) {
					break;
				}

				if (current_boost +
					    lut_coarse_gain[ref_coarse_gain] <=
				    target_boost) {
					current_boost +=
						lut_coarse_gain[ref_coarse_gain];
					ref_coarse_gain++;
					break;
				} else {
					current_boost +=
						lut_coarse_gain[ref_coarse_gain];
					ref_coarse_gain++;
				}
			}

			/* Adjust fine gain */
			if (!reached_max_fine_gain_flag) {
				i++;
				i &= TX_POWER_FINE_GAIN_MASK;
				current_boost += lut[i];
			} else {
				current_boost -= lut[i];
				i--;
				i &= TX_POWER_FINE_GAIN_MASK;
				if (i == 0)
					reached_max_fine_gain_flag = false;
			}
		}

		if (ret) {
			adj_tx_power_byte[k] = ref_tx_power_byte[k];
			current_boost = 0;
		} else {
			adj_tx_power_byte[k] = (i << 2) | ref_coarse_gain;
		}

		adj_tx_power_boost[k] = current_boost;

		if (applied_boost && (k == 0))
			*applied_boost = current_boost;
	}
	adjusted_tx_power = (u32)adj_tx_power_byte[3] << 24 |
			    (u32)adj_tx_power_byte[2] << 16 |
			    (u32)adj_tx_power_byte[1] << 8 |
			    (u32)adj_tx_power_byte[0];

	return adjusted_tx_power;
}

/**
 * dw3000_adjust_tx_power() - calculates the adjusted TxPower
 * @dw: the DW device
 * @payload_bytes: payload skbuf's length in bytes
 *
 * Wraps actual TX power calculation. Converts frame length to duration (in µs)
 * with respect to RF settings (channel, PRF, ...) then calls adjust_tx_power
 *
 * Return: the adjusted_tx_power setting than can be used to configure the chip
 * transmission level
 */
int dw3000_adjust_tx_power(struct dw3000 *dw, int payload_bytes)
{
	u16 th_boost, app_boost;
	u8 chan = dw->config.chan;
	u16 frm_dur =
		DTU_TO_US(dw3000_frame_duration_dtu(dw, payload_bytes, true));
	u32 adjusted_tx_power = adjust_tx_power(frm_dur, dw->txconfig.power,
						chan, &th_boost, &app_boost);

	trace_dw3000_adjust_tx_power(dw, dw->txconfig.power, adjusted_tx_power,
				     frm_dur, payload_bytes, chan, th_boost,
				     app_boost);

	return dw3000_set_tx_power_register(dw, adjusted_tx_power);
}
