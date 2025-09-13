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
#include "dw3000_chip_d0.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000_trc.h"

int dw3000_c0_get_dgc_dec(struct dw3000 *dw, u8 *value);
int dw3000_c0_prog_pll_coarse_code(struct dw3000 *dw);
int dw3000_c0_set_mrxlut(struct dw3000 *dw, const u32 *lut);

static const struct dw3000_chip_register d0_registers[] = {
	/* registres virtuels pour dump des fileID */
	{ "GEN_CFG0", 0x00, 0x7e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GEN_CFG1", 0x01, 0x64, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "STS_CFG", 0x02, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RX_TUNE", 0x03, 0x54, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "EXT_SYNC", 0x04, 0x21, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GPIO_CTRL", 0x05, 0x2e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DRX", 0x06, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CONF", 0x07, 0x51, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CAL", 0x08, 0x1e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "FS_CTRL", 0x09, 0x15, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "AON", 0x0a, 0x15, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "OTP_IF", 0x0b, 0x18, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_1", 0x0c, 0x6c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_2", 0x0d, 0x6c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_3", 0x0e, 0x20, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DIG_DIAG", 0x0f, 0x51, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "PMSC", 0x11, 0x4a, 0, DW3000_CHIPREG_DUMP, NULL },
	/* TODO: RX/TX buffers limited to first 128bytes only.
	   Shall we dump the whole 1024 bytes? */
	{ "RX_BUFFER", 0x12, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RX_BUFFER1", 0x13, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	/* No TX_BUFFER as read isn't supported */
	/* CIR memory require 2 bits configured elsewhere, so removed. */
	{ "SCRATCH_RAM", 0x16, 0x7f, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "AES RAM", 0x17, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "SET_X", 0x18, 0x1d0, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "IN_PTR_CFG", 0x1f, 0x12, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "euid", 0x000004, 0x08, 0x00, DW3000_CHIPREG_WP, NULL },
	{ "frame_filter", 0x000010, 0x01, 0x01, DW3000_CHIPREG_WP, NULL },
	{ "rx_phrmode", 0x000010, 0x01, 0x10, DW3000_CHIPREG_WP, NULL },
	{ "tx_phrrate", 0x000010, 0x01, 0x20, DW3000_CHIPREG_WP, NULL },
	{ "rx_sts_mode", 0x000011, 0x01, 0x8, DW3000_CHIPREG_WP, NULL },
	{ "pdoa_mode", 0x000012, 0x01, 0x3, DW3000_CHIPREG_WP, NULL },
	{ "role", 0x000015, 0x01, 0x1, DW3000_CHIPREG_WP, NULL },
	{ "frame_filter_cfg", 0x000014, 0x02, 0xfeff, DW3000_CHIPREG_WP, NULL },
	{ "datarate", 0x000021, 0x01, 0x4, DW3000_CHIPREG_WP, NULL },
	{ "tx_pream_len", 0x000021, 0x01, 0xf0, DW3000_CHIPREG_WP, NULL },
	{ "tx_antdly", 0x00007c, 0x02, 0x00, DW3000_CHIPREG_WP, NULL },
	{ "txrf_pwrfin", 0x010004, 0x01, 0xfc, DW3000_CHIPREG_WP, NULL },
	{ "tx_pwr", 0x010004, 0x04, 0x00, DW3000_CHIPREG_WP, NULL },
	{ "rx_sfdtype", 0x010008, 0x01, 0x6, DW3000_CHIPREG_WP, NULL },
	{ "channel", 0x010008, 0x01, 0x01, DW3000_CHIPREG_WP, NULL },
	{ "tx_pream_ch", 0x010008, 0x01, 0xf8, DW3000_CHIPREG_WP, NULL },
	{ "prf", 0x010008, 0x01, 0x1f, DW3000_CHIPREG_WP, NULL },
	{ "rx_phr_rate", 0x010010, 0x01, 0x20, DW3000_CHIPREG_WP, NULL },
	{ "rx_sts_len", 0x020000, 0x01, 0xff, DW3000_CHIPREG_WP, NULL },
	{ "ext_clkdly", 0x040000, 0x02, 0x7f8, DW3000_CHIPREG_WP, NULL },
	{ "ext_clkdly_en", 0x040001, 0x01, 0x8, DW3000_CHIPREG_WP, NULL },
	{ "gpiodir", 0x050008, 0x02, 0x1ff, DW3000_CHIPREG_WP, NULL },
	{ "gpioout", 0x05000c, 0x02, 0x1ff, DW3000_CHIPREG_WP, NULL },
	{ "rx_paclen", 0x060000, 0x01, 0x03, DW3000_CHIPREG_WP, NULL },
	{ "rx_sfd_to", 0x060002, 0x02, 0x00, DW3000_CHIPREG_WP, NULL },
	{ "chan_pg_delay", 0x07001c, 0x01, 0x3f, DW3000_CHIPREG_WP, NULL },
	{ "chan_pll_cfg", 0x090001, 0x01, 0x10, DW3000_CHIPREG_WP, NULL },
	{ "xtal_trim", 0x090014, 0x01, 0x3f, DW3000_CHIPREG_WP, NULL },
	{ "rx_antdly", 0x0e0000, 0x01, 0xf8, DW3000_CHIPREG_WP, NULL },
	{ "rx_diag", 0x0e0002, 0x01, 0x10, DW3000_CHIPREG_WP, NULL },
	{ "digi_diag_en", 0x0f0000, 0x01, 0x1, DW3000_CHIPREG_WP, NULL },
	{ "digi_diag_clr", 0x0f0000, 0x01, 0x2, DW3000_CHIPREG_WP, NULL },
};

const struct dw3000_chip_register *dw3000_d0_get_registers(struct dw3000 *dw,
							   size_t *count)
{
	*count = ARRAY_SIZE(d0_registers);
	return d0_registers;
}

const u32 *dw3000_d0_get_config_mrxlut_chan(struct dw3000 *dw, u8 channel)
{
	/* Lookup table default values for channel 5 */
	static const u32 dw3000_d0_configmrxlut_ch5[DW3000_CONFIGMRXLUT_MAX] = {
		0x1c0fd, 0x1c43e, 0x1c6be, 0x1c77e, 0x1cf36, 0x1cfb5, 0x1cff5
	};

	/* Lookup table default values for channel 9 */
	static const u32 dw3000_d0_configmrxlut_ch9[DW3000_CONFIGMRXLUT_MAX] = {
		0x2a8fe, 0x2ac36, 0x2a5fe, 0x2af3e, 0x2af7d, 0x2afb5, 0x2afb5
	};

	switch (channel) {
	case 5:
		return dw3000_d0_configmrxlut_ch5;
	case 9:
		return dw3000_d0_configmrxlut_ch9;
	default:
		return NULL;
	}
}

/**
 * dw3000_d0_softreset() - D0 chip specific software reset
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_softreset(struct dw3000 *dw)
{
	/* D0 require a FAST command to start soft-reset */
	return dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_RESET);
}

/**
 * dw3000_d0_init() - D0 chip specific initialisation
 * @dw: The DW device.
 *
 * Note: Still used by dw3000_e0_init().
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_init(struct dw3000 *dw)
{
	int rc = 0;

	if (dw->current_operational_state != DW3000_OP_STATE_DEEP_SLEEP) {
		/* Disable nfcc_coex mail box only if the chip is not in deep sleep.
		 * Else, on wake up, the nfcc_coex state will be not detectable. */
		rc = dw3000_nfcc_coex_disable(dw);
	}
	return rc;
}

/**
 * dw3000_d0_coex_init() - Configure the device's WiFi coexistence GPIO
 * @dw: The DW device.
 *
 * Note: Still used by dw3000_e0_coex_init() as GPIO pin need to be configured.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_coex_init(struct dw3000 *dw)
{
	u32 modemask;
	u16 dirmask;
	int rc;

	if (dw->coex_gpio < 0)
		return 0;
	/* Ensure selected GPIO is well configured */
	modemask = DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK
		   << (DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * dw->coex_gpio);
	rc = dw3000_set_gpio_mode(dw, modemask, 0);
	if (rc)
		return rc;
	dirmask = DW3000_GPIO_DIR_GDP0_BIT_MASK
		  << (DW3000_GPIO_DIR_GDP0_BIT_LEN * dw->coex_gpio);
	rc = dw3000_set_gpio_dir(dw, dirmask, 0);
	return rc;
}

/**
 * dw3000_d0_coex_gpio() - Update the device's WiFi coexistence GPIO
 * @dw: The DW device.
 * @state: The WiFi coexistence GPIO state to apply.
 * @delay_us: The delay in us before changing GPIO state.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_d0_coex_gpio(struct dw3000 *dw, bool state, int delay_us)
{
	int offset;
	/* /!\ could be called first with (true, 1000), then before end of 1000
	   microseconds could be called with (false, 0), should handle this case
	   with stopping the timer if any */
	if (delay_us) {
		/* Wait to ensure GPIO is toggle on time */
		if (delay_us > 10)
			usleep_range(delay_us - 10, delay_us);
		else
			udelay(delay_us);
	}
	offset = DW3000_GPIO_OUT_GOP0_BIT_LEN * dw->coex_gpio;
	trace_dw3000_coex_gpio(dw, state, delay_us, 0, dw->coex_status);
	dw3000_set_gpio_out(dw, !state << offset, state << offset);
	return 0;
}

static int dw3000_d0_check_tx_ok(struct dw3000 *dw)
{
	return 0;
}

/**
 * dw3000_d0_prog_ldo_and_bias_tune() - Programs the device's LDO and BIAS tuning
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_prog_ldo_and_bias_tune(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_otp_data *otp = &dw->otp_data;
	if (otp->ldo_tune_lo && otp->ldo_tune_hi) {
		dw3000_reg_or16(dw, DW3000_NVM_CFG_ID, 0, DW3000_LDO_BIAS_KICK);
		/* Save the kicks for the on-wake configuration */
		local->sleep_mode |= DW3000_LOADLDO;
	}
	/* Use DGC_CFG from OTP */
	local->dgc_otp_set = otp->dgc_addr == DW3000_DGC_CFG0 ? true : false;
	return 0;
}

/**
 * dw3000_d0_pll_calibration_from_scratch() - Calibrate the PLL from scratch
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_d0_pll_calibration_from_scratch(struct dw3000 *dw)
{
	int rc = 0;

	/* Run the PLL calibration from scratch.
	 * The USE_OLD_BIT_MASK tells the chip to use the an old PLL_CAL_ID to start
	 * its calculation. This is just in order to fasten the process.
	 */
	rc = dw3000_reg_or32(dw, DW3000_PLL_CAL_ID, 0,
			     DW3000_PLL_CAL_PLL_USE_OLD_BIT_MASK);
	if (rc)
		return rc;
	/* Wait for the PLL calibration (needed before read the calibration status register) */
	usleep_range(DW3000_D0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US,
		     DW3000_D0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US + 10);
	return rc;
}

/**
 * dw3000_d0_compute_rssi() - Compute RSSI from its composites
 * @dw: the DW device
 * @rssi: RSSI composites
 * @rx_tune: state of RX_TUNE_EN bit leads to use dgc_dec value or not
 * @sts: current sts mode
 *
 * Because RSSI cannot be positive in our case (would mean that signals have
 * been amplified) we return an unsigned integer considered as always negative.
 *
 * Return: 0 on error, else the RSSI in absolute value and as an integer.
 * expressed in dBm, Q32.0.
 */
u32 dw3000_d0_compute_rssi(struct dw3000 *dw, struct dw3000_rssi *rssi,
			   bool rx_tune, u8 sts)
{
	/* Details are logged into UWB-3455
	 * DW3700 User Manual v0.4 section 4.7.2 gives that RSSI
	 * can be computed using this formula:
	 * rssi = 10 * log10 ((cir_pwr * 2^17) / pacc_cnt ^ 2) + 6D - A(prf)
	 * But log10 isn't implemented ; let's use ilog2 instead, because it is
	 * easy to do on binary numbers. Thus, formula becomes:
	 * rssi = 3*log2(cir_pwr) - 6*log2(pacc_cnt) + 3*log2(2^17) + 6*D - A(prf)
	 * Notice that 3*log2(2^17) +6*D - A(prf) can be pre-computed.
	 * Factor 3 comes from ln(2) / ln(10) almost equal to 3 / 10 ; this is an
	 * approximation done in equation turning log10 into log2 to avoid floating point
	 */

	/* u32 is used because ilog2 macro cannot work on bitfield */
	u32 pwr = rssi->cir_pwr;
	u32 cnt = rssi->pacc_cnt;
	s32 r, rssi_constant;

	/* Do not consider bad packets */
	if (unlikely(!pwr || !cnt))
		return 0;

	rssi_constant = DW3000_RSSI_CONSTANT + 6 * rx_tune * rssi->dgc_dec;

	if (!rssi->prf_64mhz) {
		rssi_constant -= DW3000_RSSI_OFFSET_PRF16;
	} else
		rssi_constant -= ((sts == DW3000_STS_MODE_OFF) ?
					  DW3000_RSSI_OFFSET_PRF64_IPATOV :
					  DW3000_RSSI_OFFSET_PRF64_STS);

	r = 3 * ilog2(pwr) - 6 * ilog2(cnt) + rssi_constant;
	if (unlikely(r > 0)) {
		dev_err(dw->dev, "bad rssi value. Forced to 0\n");
		r = 0;
	}

	return (u32)-r;
}

const struct dw3000_chip_ops dw3000_chip_d0_ops = {
	.softreset = dw3000_d0_softreset,
	.init = dw3000_d0_init,
	.coex_init = dw3000_d0_coex_init,
	.coex_gpio = dw3000_d0_coex_gpio,
	.check_tx_ok = dw3000_d0_check_tx_ok,
	.prog_ldo_and_bias_tune = dw3000_d0_prog_ldo_and_bias_tune,
	.get_config_mrxlut_chan = dw3000_d0_get_config_mrxlut_chan,
	.get_dgc_dec = dw3000_c0_get_dgc_dec,
	.pll_calibration_from_scratch = dw3000_d0_pll_calibration_from_scratch,
	.prog_pll_coarse_code = dw3000_c0_prog_pll_coarse_code,
	.set_mrxlut = dw3000_c0_set_mrxlut,
	.get_registers = dw3000_d0_get_registers,
	.compute_rssi = dw3000_d0_compute_rssi,
};
