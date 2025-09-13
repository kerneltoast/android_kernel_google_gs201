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
#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_core_reg.h"
#include "dw3000_chip_e0.h"
#include "dw3000_trc.h"

int dw3000_c0_prog_pll_coarse_code(struct dw3000 *dw);
int dw3000_d0_softreset(struct dw3000 *dw);
int dw3000_d0_init(struct dw3000 *dw);
int dw3000_d0_coex_init(struct dw3000 *dw);
const struct dw3000_chip_register *dw3000_d0_get_registers(struct dw3000 *dw,
							   size_t *count);
u32 dw3000_d0_compute_rssi(struct dw3000 *dw, struct dw3000_rssi *rssi,
			   bool rx_tune, u8 sts);

const u32 *dw3000_e0_get_config_mrxlut_chan(struct dw3000 *dw, u8 channel)
{
	/* Lookup table default values for channel 5 */
	static const u32 dw3000_e0_configmrxlut_ch5[DW3000_CONFIGMRXLUT_MAX] = {
		0x3803e, 0x3876e, 0x397fe, 0x38efe, 0x39c7e, 0x39dfe, 0x39ff6
	};

	/* Lookup table default values for channel 9 */
	static const u32 dw3000_e0_configmrxlut_ch9[DW3000_CONFIGMRXLUT_MAX] = {
		0x5407e, 0x547be, 0x54d36, 0x55e36, 0x55f36, 0x55df6, 0x55ffe
	};

	switch (channel) {
	case 5:
		return dw3000_e0_configmrxlut_ch5;
	case 9:
		return dw3000_e0_configmrxlut_ch9;
	default:
		return NULL;
	}
}

/**
 * DW3000_COEX_TIMER_XTAL - Timer frequency to use for WiFi coexistence
 *
 * Max delay is 1s during ranging (1Hz), so require to use per 64 divisor
 * to ensure that calculated expire value is lower than 21bits, the max
 * register value.
 */
#define DW3000_COEX_TIMER_XTAL DW3000_TIMER_XTAL_DIV64

/**
 * dw3000_e0_init() - E0 chip specific initialisation
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_init(struct dw3000 *dw)
{
	int rc = dw3000_d0_init(dw);
	if (rc)
		return rc;
	/* Ensure GPIO block clock is enabled */
	return dw3000_reg_or8(dw, DW3000_CLK_CTRL_ID, 2,
			      DW3000_CLK_CTRL_GPIO_CLK_EN_BIT_MASK >> 16);
}

/**
 * dw3000_e0_coex_init() - Configure the device's WiFi coexistence GPIO
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_coex_init(struct dw3000 *dw)
{
	struct dw3000_timer_cfg cfg = { .divider = DW3000_COEX_TIMER_XTAL,
					.mode = DW3000_SINGLE_MODE,
					.gpio_stop = 0,
					.coex_out = 1 };
	int rc;
	u8 tmp;

	if (dw->coex_gpio < 0)
		return 0;
	/* Validate configured WiFi coex GPIO */
	if (dw->coex_gpio < 4 || dw->coex_gpio > 5) {
		/* Disable if badly configured and return an error */
		dw->coex_gpio = -1;
		return -EINVAL;
	}
	/* Ensure selected GPIO clock is enabled */
	rc = dw3000_reg_or8(dw, DW3000_CLK_CTRL_ID, 2,
			    DW3000_CLK_CTRL_GPIO_CLK_EN_BIT_MASK >> 16);
	if (rc)
		return rc;
	/* Ensure selected GPIO is well configured, same as D0 chip */
	rc = dw3000_d0_coex_init(dw);
	if (rc)
		return rc;
	/* Swap COEX GPIO if need to use GPIO 4 as COEX_OUT */
	if (dw->coex_gpio == 4) {
		tmp = DW3000_GPIO_MODE_COEX_IO_SWAP_BIT_MASK >> 24;
		rc = dw3000_reg_or8(dw, DW3000_GPIO_MODE_ID, 3, tmp);
	} else {
		tmp = (u8)(~DW3000_GPIO_MODE_COEX_IO_SWAP_BIT_MASK >> 24);
		rc = dw3000_reg_and8(dw, DW3000_GPIO_MODE_ID, 3, tmp);
	}
	if (rc)
		return rc;

	/* Configure E0 timer0 for use */
	rc = dw3000_timers_enable(dw);
	if (rc)
		return rc;
	rc = dw3000_timers_reset(dw);
	if (rc)
		return rc;
	rc = dw3000_timer_configure(dw, DW3000_TIMER0, &cfg);
	if (rc)
		return rc;
	return 0;
}

/**
 * dw3000_e0_coex_gpio() - Update the device's WiFi coexistence GPIO
 * @dw: The DW device.
 * @state: The WiFi coexistence GPIO state to apply.
 * @delay_us: The delay in us before changing GPIO state.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_coex_gpio(struct dw3000 *dw, bool state, int delay_us)
{
	int offset = DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * dw->coex_gpio;
	u32 modemask = DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK << offset;
	int rc;
	/* /!\ could be called first with (true, 1000), then before end of 1000
	   microseconds could be called with (false, 0), should handle this case
	   with stopping the timer if any */
	if (delay_us) {
		const int factor =
			(DW3000_TIMER_FREQ >> DW3000_COEX_TIMER_XTAL) / 1000;
		const int divisor = 1000000 / 1000;
		u32 expire;
		/* Reconfigure selected GPIO for COEX mode */
		rc = dw3000_set_gpio_mode(dw, modemask, 1 << offset);
		if (rc)
			return rc;
		/* Re-configure COEX_OUT_MODE in SYS_CFG for Timer use */
		rc = dw3000_reg_or32(dw, DW3000_SYS_CFG_ID, 0,
				     DW3000_SYS_CFG_COEX_OUT_MODE_BIT_MASK);
		if (rc)
			return rc;
		/* Launch timer0 */
		expire = delay_us * factor / divisor;
		rc = dw3000_timer_set_expiration(dw, DW3000_TIMER0, expire);
		if (rc)
			return rc;
		trace_dw3000_coex_gpio(dw, state, delay_us, expire,
				       dw->coex_status);
		return dw3000_timer_start(dw, DW3000_TIMER0);
	}
	if (!state) {
		/* Stop/reset timer0 & 1 */
		rc = dw3000_timers_reset(dw);
		if (rc)
			return rc;
		/* Reset COEX_OUT_MODE in SYS_CFG to 0 */
		rc = dw3000_reg_and32(
			dw, DW3000_SYS_CFG_ID, 0,
			~(u32)DW3000_SYS_CFG_COEX_OUT_MODE_BIT_MASK);
		if (rc)
			return rc;
	}
	/* Reconfigure COEX GPIO for GPIO mode */
	rc = dw3000_set_gpio_mode(dw, modemask, 0);
	if (rc)
		return rc;
	/* Update GPIO output state */
	offset = DW3000_GPIO_OUT_GOP0_BIT_LEN * dw->coex_gpio;
	trace_dw3000_coex_gpio(dw, state, delay_us, 0, dw->coex_status);
	return dw3000_set_gpio_out(dw, !state << offset, state << offset);
}

static int dw3000_e0_check_tx_ok(struct dw3000 *dw)
{
	return 0;
}

/**
 * dw3000_e0_prog_ldo_and_bias_tune() - Programs the device's LDO and BIAS tuning
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_e0_prog_ldo_and_bias_tune(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_otp_data *otp = &dw->otp_data;

	if (otp->ldo_tune_lo && otp->ldo_tune_hi && otp->bias_tune) {
		dw3000_reg_or16(dw, DW3000_NVM_CFG_ID, 0, DW3000_LDO_BIAS_KICK);
		/* Save the kicks for the on-wake configuration */
		local->sleep_mode |= DW3000_LOADLDO | DW3000_LOADBIAS;
	}
	/* Ignore use of DGC from OTP */
	local->dgc_otp_set = false;
	return 0;
}

/**
 * dw3000_timers_enable() - Enables the device's timers block
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timers_enable(struct dw3000 *dw)
{
	/* Enable LDO to run the timer - needed if not in IDLE state */
	return dw3000_reg_or8(dw, DW3000_LDO_CTRL_ID, 0,
			      DW3000_LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK);
}

/**
 * dw3000_timers_reset() - Reset the device's timers block
 * @dw: the DW device
 *
 * It will reset both timers. It can be used to stop a timer running in repeat
 * mode.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timers_reset(struct dw3000 *dw)
{
	return dw3000_reg_and16(dw, DW3000_SOFT_RST_ID, 0,
				(u16)(~DW3000_SOFT_RST_TIM_RST_N_BIT_MASK));
}

/**
 * dw3000_timers_read_and_clear_events() - Read the timers' event counts
 * @dw: the DW device
 * @evt0: pointer where to store timer0's event count
 * @evt1: pointer where to store timer1's event count
 *
 * When reading from this register the values will be reset/cleared, thus the
 * host needs to read both timers' event counts.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timers_read_and_clear_events(struct dw3000 *dw, u8 *evt0, u8 *evt1)
{
	int rc;
	u16 status;
	rc = dw3000_reg_read16(dw, DW3000_TIMER_STATUS_ID, 0, &status);
	if (unlikely(rc))
		return rc;
	if (evt0)
		*evt0 = (u8)status;
	if (evt1)
		*evt1 = (u8)(status >> 8);
	return 0;
}

/**
 * dw3000_timer_configure() - Configures the selected timer
 * @dw: the DW device
 * @timer: timer to configure
 * @cfg: pointer to structure holding timer configuration
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timer_configure(struct dw3000 *dw, enum dw3000_timer timer,
			   struct dw3000_timer_cfg *cfg)
{
	u32 config =
		((u16)cfg->divider
		 << DW3000_TIMER_CTRL_TIMER_0_DIV_BIT_OFFSET) |
		((u16)cfg->mode << DW3000_TIMER_CTRL_TIMER_0_MODE_BIT_OFFSET) |
		((u16)cfg->gpio_stop
		 << DW3000_TIMER_CTRL_TIMER_0_GPIO_BIT_OFFSET) |
		((u16)cfg->coex_out
		 << DW3000_TIMER_CTRL_TIMER_0_COEXOUT_BIT_OFFSET);
	/* For TIMER 1 we write the configuration at offset 2 */
	config <<= (u8)timer * 8;
	/* Ensure reading CNT register return current counter value! */
	config |= DW3000_TIMER_CTRL_TIMER_0_RD_COUNT_BIT_MASK << (u8)timer;
	return dw3000_reg_write32(dw, DW3000_TIMER_CTRL_ID, 0, config);
}

/**
 * dw3000_timer_set_expiration() - sets timer expiration delay
 * @dw: the DW device
 * @timer: timer to set expiration period (see enum dw3000_timer)
 * @exp: expiry count in timer frequency unit
 *
 * This function set the 22 lower bits of expiration period.
 * It take expiry count in timer frequency unit, e.g. if units are XTAL/64 (1.66 us)
 * then setting 1024 ~= 1.7 ms delay.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timer_set_expiration(struct dw3000 *dw, enum dw3000_timer timer,
				u32 exp)
{
	u32 reg = DW3000_TIMER0_CNT_SET_ID + (4 * timer);
	return dw3000_reg_write32(
		dw, reg, 0, exp & DW3000_TIMER0_CNT_SET_TIMER_0_SET_BIT_MASK);
}

/**
 * dw3000_timer_get_counter() - retrieve timer counter or expiration delay
 * @dw: the DW device
 * @timer: timer to set expiration period (see enum dw3000_timer)
 * @counter: pointer where current counter value or expiry count is saved
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timer_get_counter(struct dw3000 *dw, enum dw3000_timer timer,
			     u32 *counter)
{
	u32 reg = DW3000_TIMER0_CNT_SET_ID + (4 * timer);
	return dw3000_reg_read32(dw, reg, 0, counter);
}

/**
 * dw3000_timer_start() - Enables the specified timer
 * @dw: the DW device
 * @timer: timer to enable
 *
 * In order to enable, the timer enable bit [0] for TIMER0 or [1] for TIMER1
 * needs to transition from 0 -> 1.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_timer_start(struct dw3000 *dw, enum dw3000_timer timer)
{
	/* TODO: check if dw3000_reg_modify8() is working or not */
	u8 val = 1 << timer;
	int rc;
	/* Set to '0' */
	rc = dw3000_reg_and8(dw, DW3000_TIMER_CTRL_ID, 0, ~val);
	if (rc)
		return rc;
	/* Set to '1' */
	return dw3000_reg_or8(dw, DW3000_TIMER_CTRL_ID, 0, val);
}

/**
 * dw3000_e0_adc_calibration_monitor_thresholds() - Monitors the thresholds
 * @dw: the DW device
 * @thresholds: the monitored thresholds
 * @rx_event: true if any RX event triggered, else false
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_adc_calibration_monitor_thresholds(struct dw3000 *dw,
							u32 *thresholds,
							bool *rx_event)
{
	int rc, i;
	u8 status;
	u16 threshold_arr[4] = { 0, 0, 0, 0 };

	/* Monitor thresholds */
	for (i = 0; i < DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS; i++) {
		/* Unfreeze */
		rc = dw3000_reg_modify8(dw, DW3000_MRX_CFG_ID, 0, 0xFE, 0);
		if (rc)
			return rc;
		/* Freeze */
		rc = dw3000_reg_modify8(dw, DW3000_MRX_CFG_ID, 0, 0xFE, 1);
		if (rc)
			return rc;
		rc = dw3000_reg_read32(dw, DW3000_ADC_THRESH_DBG_ID, 0,
				       thresholds);
		if (rc)
			return rc;
		threshold_arr[0] += (*thresholds & 0xFF);
		threshold_arr[1] += ((*thresholds >> 8) & 0xFF);
		threshold_arr[2] += ((*thresholds >> 16) & 0xFF);
		threshold_arr[3] += ((*thresholds >> 24) & 0xFF);
	}
	threshold_arr[0] =
		(u8)(threshold_arr[0] / DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS);
	threshold_arr[1] =
		(u8)(threshold_arr[1] / DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS);
	threshold_arr[2] =
		(u8)(threshold_arr[2] / DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS);
	threshold_arr[3] =
		(u8)(threshold_arr[3] / DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS);
	*thresholds = (threshold_arr[3] << 24) + (threshold_arr[2] << 16) +
		      (threshold_arr[1] << 8) + threshold_arr[0];
	/* Once thresholds are monitored read the sys status */
	rc = dw3000_reg_read8(dw, DW3000_SYS_STATUS_ID, 0, &status);
	if (rc)
		return rc;
	/* Check and return if any RX event triggered. */
	*rx_event = !!(status & (DW3000_SYS_STATUS_ALL_RX_GOOD |
				 DW3000_SYS_STATUS_ALL_RX_ERR |
				 DW3000_SYS_STATUS_ALL_RX_TO));
	return 0;
}

/**
 * dw3000_e0_adc_offset_calibration() - Calibrate ADC
 * @dw: the DW device
 *
 * TODO: This function comes from directly from Qorvo/Decawave.
 * Hard coded values come with no documentation and should be converted into
 * define when the documentation will be available.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_e0_adc_offset_calibration(struct dw3000 *dw)
{
	int rc, k;
	u32 switch_control_reg_backup;
	u32 sys_enable_lo;
	u32 sys_enable_hi;
	u32 agc_reg_backup;
	u32 dgc_reg_backup;
	u32 dgc_lut0_reg_backup;
	u32 dgc_lut6_reg_backup;
	u8 pgf_idx;
	u32 thresholds;
	bool rx_event;

	/* Step 1: Get the current registers value used or modified by the calibration */
	rc = dw3000_reg_read32(dw, DW3000_RF_SWITCH_CTRL_ID, 0,
			       &switch_control_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_AGC_CFG_ID, 0, &agc_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_DGC_CFG_ID, 0, &dgc_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_DGC_LUT_0_CFG_ID, 0,
			       &dgc_lut0_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_DGC_LUT_6_CFG_ID, 0,
			       &dgc_lut6_reg_backup);
	if (rc)
		return rc;

	/* Step 2a: De-sensitise RX path by shunting the TXRX switch */
	rc = dw3000_reg_modify32(
		dw, DW3000_RF_SWITCH_CTRL_ID, 0,
		~DW3000_RF_SWITCH_CTRL_TXRX_SW_OVR_CTRL_BIT_MASK,
		(0x38 << DW3000_RF_SWITCH_CTRL_TXRX_SW_OVR_CTRL_BIT_OFFSET) |
			DW3000_RF_SWITCH_CTRL_TXRX_SW_OVR_EN_BIT_MASK);
	if (rc)
		return rc;
	/* Further de-sensitise the RX path by selecting a higher DGC setting */
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_0_CFG_ID, 0,
				dgc_lut6_reg_backup);
	if (rc)
		return rc;
	/* Step 2b: Disable AGC and set PGF gain manually */
	pgf_idx = dgc_lut0_reg_backup & 0x7;
	rc = dw3000_reg_modify8(dw, DW3000_AGC_CFG_ID, 0, ~(0x40 | 0x38 | 0x1),
				(0x40 | (pgf_idx << 3)));
	if (rc)
		return rc;
	/* Step 2c: ADC independent thresholds */
	rc = dw3000_reg_write8(dw, DW3000_AGC_CFG_ID, 3, 0);
	if (rc)
		return rc;
	/* Step 2d: Disable DGC */
	dw3000_reg_and8(dw, DW3000_DGC_CFG_ID, 0x0,
			(u8)~DW3000_DGC_CFG_RX_TUNE_EN_BIT_MASK);
	if (rc)
		return rc;

	/* 2e: Disable interrupt events */
	rc = dw3000_reg_read32(dw, DW3000_SYS_ENABLE_HI_ID, 0, &sys_enable_hi);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_SYS_ENABLE_LO_ID, 0, &sys_enable_lo);
	if (rc)
		return rc;
	/* disable interrupts */
	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_LO_ID, 0, 0);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_HI_ID, 0, 0);
	if (rc)
		return rc;

	/* Step 3a: Enable RX (may need further work) */
	for (k = 0; k < 2; k++) {
		{
			/* Ensure coex is disable to have immediate RX */
			s8 gpio = dw->coex_gpio;
			dw->coex_gpio = -1;
			barrier();
			rc = dw3000_rx_enable(dw, 0, 0, 0);
			dw->coex_gpio = gpio;
			if (rc)
				return rc;
		}
		usleep_range(DW3000_E0_ADC_CALIBRATION_DELAY_US,
			     DW3000_E0_ADC_CALIBRATION_DELAY_US + 10);

		/* Step 3b: monitor thresholds */
		rc = dw3000_e0_adc_calibration_monitor_thresholds(
			dw, &thresholds, &rx_event);
		if (rc)
			return rc;

		/* Step 3c: disable receiver */
		{
			/* Ensure coex is disable  */
			s8 gpio = dw->coex_gpio;
			dw->coex_gpio = -1;
			barrier();
			rc = dw3000_forcetrxoff(dw);
			dw->coex_gpio = gpio;
			if (rc)
				return rc;
		}

		if (!rx_event)
			break;
	}

	/* 3d: restore interrupts */
	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_HI_ID, 0, sys_enable_hi);
	if (rc)
		return rc;

	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_LO_ID, 0, sys_enable_lo);
	if (rc)
		return rc;

	/* Step 3e: Set initial DAC indices to settled RMS values */
	rc = dw3000_reg_write32(dw, DW3000_ADC_THRESH_CFG_ID, 0, thresholds);
	if (rc)
		return rc;
	/* Step 4: restore initial register values */
	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_HI_ID, 0, sys_enable_hi);
	if (rc)
		return rc;

	rc = dw3000_reg_write32(dw, DW3000_SYS_ENABLE_LO_ID, 0, sys_enable_lo);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_RF_SWITCH_CTRL_ID, 0,
				switch_control_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_AGC_CFG_ID, 0, agc_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG_ID, 0, dgc_reg_backup);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_0_CFG_ID, 0,
				dgc_lut0_reg_backup);
	if (rc)
		return rc;
	if (rx_event)
		return -EINVAL;
	return 0;
}

/**
 * dw3000_e0_pll_calibration_from_scratch() - Calibrate the PLL from scratch
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_pll_calibration_from_scratch(struct dw3000 *dw)
{
	int rc = 0;

	/* Run the PLL calibration from scratch.
	 * The USE_OLD_BIT_MASK tells the chip to use the an old PLL_CAL_ID to start
	 * its calculation. This is just in order to fasten the process.
	 */
	rc = dw3000_reg_or32(dw, DW3000_PLL_CAL_ID, 0,
			     DW3000_PLL_CAL_PLL_CAL_EN_BIT_MASK |
				     DW3000_PLL_CAL_PLL_USE_OLD_BIT_MASK);
	if (rc)
		return rc;
	/* Wait for the PLL calibration (needed before read the calibration status register) */
	usleep_range(DW3000_E0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US,
		     DW3000_E0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US + 10);
	return rc;
}

/**
 * dw3000_e0_get_dgc_dec() - Read DGC_DBG register content
 * @dw: The DW device.
 * @value: Pointer to store DGC DECISION value.
 *
 * Return: zero on succes, else a negative error code.
 */
static int dw3000_e0_get_dgc_dec(struct dw3000 *dw, u8 *value)
{
	int rc;
	u32 dgc_dbg;

	rc = dw3000_reg_read32(dw, DW3000_E0_DGC_DBG_ID, 0, &dgc_dbg);
	if (unlikely(rc))
		return rc;

	/* DGC_DECISION is on bits 28 to 30 of DGC_CFG, cf 8.2.4.2 of DW3700
	 * User Manual, store it to right the rssi stats entry */
	*value = (u8)((dgc_dbg & 0x70000000) >> 0x1c);
	return 0;
}

/**
 * dw3000_e0_pll_coarse_code() - Calibrate the PLL from scratch
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_pll_coarse_code(struct dw3000 *dw)
{
	u8 tmp;

	/* Clear CH9_ICAS/RCAS bits */
	tmp = (u8)(~(DW3000_PLL_COARSE_CODE_CH9_ICAS_BIT_MASK |
		     DW3000_PLL_COARSE_CODE_CH9_RCAS_BIT_MASK) >>
		   24);
	return dw3000_reg_and8(dw, DW3000_PLL_COARSE_CODE_ID, 3, tmp);
}

/**
 * dw3000_e0_set_mrxlut() - Configure mrxlut
 * @dw: The DW device.
 * @lut: Pointer to LUT to write to DGC_LUT_X_CFG registers
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_e0_set_mrxlut(struct dw3000 *dw, const u32 *lut)
{
	int rc;

	/* Update DGC CFG. Leave it at the beginning for E0 chip */
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG0_ID, 0x0, DW3000_DGC_CFG0);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG1_ID, 0x0, DW3000_DGC_CFG1);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG2_ID, 0x0, DW3000_DGC_CFG2);
	if (rc)
		return rc;

	/* Update LUT registers */
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_0_CFG_ID, 0x0, lut[0]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_1_CFG_ID, 0x0, lut[1]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_2_CFG_ID, 0x0, lut[2]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_3_CFG_ID, 0x0, lut[3]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_4_CFG_ID, 0x0, lut[4]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_5_CFG_ID, 0x0, lut[5]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_6_CFG_ID, 0x0, lut[6]);
	return rc;
}

const struct dw3000_chip_ops dw3000_chip_e0_ops = {
	.softreset = dw3000_d0_softreset,
	.init = dw3000_e0_init,
	.coex_init = dw3000_e0_coex_init,
	.coex_gpio = dw3000_e0_coex_gpio,
	.check_tx_ok = dw3000_e0_check_tx_ok,
	.prog_ldo_and_bias_tune = dw3000_e0_prog_ldo_and_bias_tune,
	.get_config_mrxlut_chan = dw3000_e0_get_config_mrxlut_chan,
	.get_dgc_dec = dw3000_e0_get_dgc_dec,
	.adc_offset_calibration = dw3000_e0_adc_offset_calibration,
	.pll_calibration_from_scratch = dw3000_e0_pll_calibration_from_scratch,
	.prog_pll_coarse_code = dw3000_c0_prog_pll_coarse_code,
	.pll_coarse_code = dw3000_e0_pll_coarse_code,
	.set_mrxlut = dw3000_e0_set_mrxlut,
	.get_registers = dw3000_d0_get_registers,
	.compute_rssi = dw3000_d0_compute_rssi,
};
