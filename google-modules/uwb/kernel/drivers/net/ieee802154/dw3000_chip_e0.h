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
#ifndef __DW3000_CHIP_E0_H
#define __DW3000_CHIP_E0_H

/* Forward declaration */
struct dw3000;

enum dw3000_timer { DW3000_TIMER0 = 0, DW3000_TIMER1 };

enum dw3000_timer_mode { DW3000_SINGLE_MODE = 0, DW3000_REPEAT_MODE };

/* Register PLL_COARSE_CODE */
#define DW3000_PLL_COARSE_CODE_ID 0x90004
#define DW3000_PLL_COARSE_CODE_LEN (4U)
#define DW3000_PLL_COARSE_CODE_MASK 0xFFFFFFFFUL
#define DW3000_PLL_COARSE_CODE_CH9_CAL_WITH_PREBUF_BIT_OFFSET (27U)
#define DW3000_PLL_COARSE_CODE_CH9_CAL_WITH_PREBUF_BIT_LEN (1U)
#define DW3000_PLL_COARSE_CODE_CH9_CAL_WITH_PREBUF_BIT_MASK 0x8000000UL
#define DW3000_PLL_COARSE_CODE_CH5_CAL_WITH_PREBUF_BIT_OFFSET (26U)
#define DW3000_PLL_COARSE_CODE_CH5_CAL_WITH_PREBUF_BIT_LEN (1U)
#define DW3000_PLL_COARSE_CODE_CH5_CAL_WITH_PREBUF_BIT_MASK 0x4000000UL
#define DW3000_PLL_COARSE_CODE_CH9_ICAS_BIT_OFFSET (25U)
#define DW3000_PLL_COARSE_CODE_CH9_ICAS_BIT_LEN (1U)
#define DW3000_PLL_COARSE_CODE_CH9_ICAS_BIT_MASK 0x2000000UL
#define DW3000_PLL_COARSE_CODE_CH9_RCAS_BIT_OFFSET (24U)
#define DW3000_PLL_COARSE_CODE_CH9_RCAS_BIT_LEN (1U)
#define DW3000_PLL_COARSE_CODE_CH9_RCAS_BIT_MASK 0x1000000UL
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_OFFSET (8U)
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_LEN (14U)
#define DW3000_PLL_COARSE_CODE_CH5_VCO_COARSE_TUNE_BIT_MASK 0x3fff00UL
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_OFFSET (0U)
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_LEN (7U)
#define DW3000_PLL_COARSE_CODE_CH9_VCO_COARSE_TUNE_BIT_MASK 0x7fU

/* Delay to wait after RX enable to calibrate the ADC on E0 chip */
#define DW3000_E0_ADC_CALIBRATION_DELAY_US (200)

/* Loops to compute the ADC threshold average on E0 chip */
#define DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS (4)
/* Time to wait before reading the calibration status register
 * when a calibration from scratch is executed */
#define DW3000_E0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US (400)

#define DW3000_TIMER_FREQ 38400000

#define DW3000_E0_DGC_DBG_ID 0x30054

enum dw3000_timer_period {
	/* 38.4 MHz */
	DW3000_TIMER_XTAL_NODIV = 0,
	/* 19.2 MHz */
	DW3000_TIMER_XTAL_DIV2,
	/* 9.6 MHz */
	DW3000_TIMER_XTAL_DIV4,
	/* 4.8 MHz */
	DW3000_TIMER_XTAL_DIV8,
	/* 2.4 MHz */
	DW3000_TIMER_XTAL_DIV16,
	/* 1.2 MHz */
	DW3000_TIMER_XTAL_DIV32,
	/* 0.6 MHz */
	DW3000_TIMER_XTAL_DIV64,
	/* 0.3 MHz */
	DW3000_TIMER_XTAL_DIV128
};

struct dw3000_timer_cfg {
	/* Select the timer frequency (divider) */
	enum dw3000_timer_period divider;
	/* Select the timer mode */
	enum dw3000_timer_mode mode;
	/* Set to '1' to halt this timer on GPIO interrupt */
	u8 gpio_stop;
	/* Configure GPIO for WiFi co-ex */
	u8 coex_out;
};

/* Hardware timer functions */
int dw3000_timers_enable(struct dw3000 *dw);
int dw3000_timers_reset(struct dw3000 *dw);
int dw3000_timers_read_and_clear_events(struct dw3000 *dw, u8 *evt0, u8 *evt1);

int dw3000_timer_configure(struct dw3000 *dw, enum dw3000_timer timer,
			   struct dw3000_timer_cfg *cfg);
int dw3000_timer_set_expiration(struct dw3000 *dw, enum dw3000_timer timer,
				u32 exp);
int dw3000_timer_get_counter(struct dw3000 *dw, enum dw3000_timer timer,
			     u32 *counter);
int dw3000_timer_start(struct dw3000 *dw, enum dw3000_timer timer);

#endif /* __DW3000_CHIP_E0_H */
