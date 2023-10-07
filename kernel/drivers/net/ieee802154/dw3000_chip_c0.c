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
#include "dw3000_chip_c0.h"

#define DW3000_C0_DGC_DBG_ID 0x30060

static const struct dw3000_chip_register c0_registers[] = {
	/* virtual registers for fileID dump */
	{ "GEN_CFG0", 0x00, 0x79, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GEN_CFG1", 0x01, 0x64, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "STS_CFG", 0x02, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	/* No fileID 0x03 documented in DW3000 user manual v0.7. */
	{ "EXT_SYNC", 0x04, 0x04, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GPIO_CTRL", 0x05, 0x2e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DRX_CONF", 0x06, 0x10, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CONF", 0x07, 0x4c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "TX_CAL", 0x08, 0x1e, 0, DW3000_CHIPREG_DUMP, NULL },
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

const struct dw3000_chip_register *dw3000_c0_get_registers(struct dw3000 *dw,
							   size_t *count)
{
	*count = ARRAY_SIZE(c0_registers);
	return c0_registers;
}

const u32 *dw3000_c0_get_config_mrxlut_chan(struct dw3000 *dw, u8 channel)
{
	/* Lookup table default values for channel 5 */
	static const u32 dw3000_c0_configmrxlut_ch5[DW3000_CONFIGMRXLUT_MAX] = {
		0x1c0fd, 0x1c43e, 0x1c6be, 0x1c77e, 0x1cf36, 0x1cfb5, 0x1cff5
	};

	/* Lookup table default values for channel 9 */
	static const u32 dw3000_c0_configmrxlut_ch9[DW3000_CONFIGMRXLUT_MAX] = {
		0x2a8fe, 0x2ac36, 0x2a5fe, 0x2af3e, 0x2af7d, 0x2afb5, 0x2afb5
	};

	switch (channel) {
	case 5:
		return dw3000_c0_configmrxlut_ch5;
	case 9:
		return dw3000_c0_configmrxlut_ch9;
	default:
		return NULL;
	}
}

static int dw3000_c0_softreset(struct dw3000 *dw)
{
	/* Reset HIF, TX, RX and PMSC */
	return dw3000_reg_write8(dw, DW3000_SOFT_RST_ID, 0, DW3000_RESET_ALL);
}

static int dw3000_c0_init(struct dw3000 *dw)
{
	/* TODO */
	return 0;
}

static int dw3000_c0_coex_init(struct dw3000 *dw)
{
	/* TODO */
	return 0;
}

static int dw3000_c0_coex_gpio(struct dw3000 *dw, bool state, int delay_us)
{
	/* TODO */
	return 0;
}

static int dw3000_c0_check_tx_ok(struct dw3000 *dw)
{
	u32 state;
	int rc = dw3000_reg_read32(dw, DW3000_SYS_STATE_LO_ID, 0, &state);
	if (rc || state == DW3000_SYS_STATE_TXERR) {
		dw3000_forcetrxoff(dw);
		return -ETIME;
	}
	return 0;
}

/**
 * dw3000_c0_prog_ldo_and_bias_tune() - Programs the device's LDO and BIAS tuning
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_prog_ldo_and_bias_tune(struct dw3000 *dw)
{
	const u16 bias_mask = DW3000_BIAS_CTRL_DIG_BIAS_DAC_ULV_BIT_MASK;
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_otp_data *otp = &dw->otp_data;
	int rc;
	u16 bias_tune = (otp->bias_tune >> 16) & bias_mask;
	if (otp->ldo_tune_lo && otp->ldo_tune_hi && bias_tune) {
		rc = dw3000_reg_or16(dw, DW3000_NVM_CFG_ID, 0,
				     DW3000_LDO_BIAS_KICK);
		if (rc)
			return rc;
		rc = dw3000_reg_modify16(dw, DW3000_BIAS_CTRL_ID, 0, ~bias_mask,
					 bias_tune);
		if (rc)
			return rc;
	}
	local->dgc_otp_set = false;
	return 0;
}

/**
 * dw3000_c0_pre_read_sys_time() - Ensure SYS_TIME register is cleared
 * @dw: The DW device.
 *
 * On C0 chips, the SYS_TIME register value is latched and any subsequent read
 * will return the same value. To clear the current value in the register an SPI
 * write transaction is necessary, the following read of the SYS_TIME register will
 * return a new value.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_pre_read_sys_time(struct dw3000 *dw)
{
	/* The SPI_COLLISION register is choose to make this SPI write
	 * transaction because it is unused and it is a small 8 bits register.
	 */
	return dw3000_clear_spi_collision_status(
		dw, DW3000_SPI_COLLISION_STATUS_BIT_MASK);
}

/**
 * dw3000_c0_get_dgc_dec() - Read DGC_DBG register content
 * @dw: The DW device.
 * @value: Pointer to store DGC DECISION value.
 *
 * Return: zero on succes, else a negative error code.
 */
int dw3000_c0_get_dgc_dec(struct dw3000 *dw, u8 *value)
{
	int rc;
	u32 dgc_dbg;

	rc = dw3000_reg_read32(dw, DW3000_C0_DGC_DBG_ID, 0, &dgc_dbg);
	if (unlikely(rc))
		return rc;

	/* DGC_DECISION is on bits 28 to 30 of DGC_CFG, cf 8.2.4.2 of DW3700
	 * User Manual, store it to right the rssi stats entry */
	*value = (u8)((dgc_dbg & 0x70000000) >> 0x1c);
	return 0;
}

/**
 * dw3000_c0_pll_calibration_from_scratch() - Calibrate the PLL from scratch
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_pll_calibration_from_scratch(struct dw3000 *dw)
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
	usleep_range(DW3000_C0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US,
		     DW3000_C0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US + 10);
	return rc;
}

/**
 * dw3000_c0_prog_pll_coarse_code() - Programs the device's coarse code
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_c0_prog_pll_coarse_code(struct dw3000 *dw)
{
	struct dw3000_otp_data *otp = &dw->otp_data;
	int rc = 0;

	if (otp->pll_coarse_code) {
		/* set the coarse code value as read from OTP */
		rc = dw3000_reg_write8(dw, DW3000_PLL_COARSE_CODE_ID, 0,
				       otp->pll_coarse_code);
	}
	return rc;
}

/**
 * dw3000_c0_set_mrxlut() - Configure mrxlut
 * @dw: The DW device.
 * @lut: Pointer to LUT to write to DGC_LUT_X_CFG registers
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_c0_set_mrxlut(struct dw3000 *dw, const u32 *lut)
{
	int rc;

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
	if (rc)
		return rc;

	/* Update DGC CFG. Leave this at end for C0/D0 chip. */
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG0_ID, 0x0, DW3000_DGC_CFG0);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG1_ID, 0x0, DW3000_DGC_CFG1);
	return rc;
}

/**
 * dw3000_c0_kick_ops_table_on_wakeup() - kick the OPS table
 * @dw: The DW device.
 *
 * kick the desired operating parameter set (OPS) table upon wakeup from sleep
 * It will load the required OPS table configuration based upon what OPS table
 * was set to be used
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_kick_ops_table_on_wakeup(struct dw3000 *dw)
{
	return 0;
}

/**
 * dw3000_c0_kick_dgc_on_wakeup() - kick the DGC
 * @dw: The DW device.
 *
 * kick the DGC upon wakeup from sleep
 * It will load the required DGC configuration from OTP based upon what channel was set to be used
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_kick_dgc_on_wakeup(struct dw3000 *dw)
{
	u8 channel = dw->config.chan;
	u16 dgc_sel = (channel == 5 ? 0 : DW3000_NVM_CFG_DGC_SEL_BIT_MASK);

	/* The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9 */
	return dw3000_reg_modify16(dw, DW3000_NVM_CFG_ID, 0,
				   (u16) ~(DW3000_NVM_CFG_DGC_SEL_BIT_MASK),
				   dgc_sel | DW3000_NVM_CFG_DGC_KICK_BIT_MASK);
}

/**
 * dw3000_c0_compute_rssi() - Compute RSSI from its composites
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
static u32 dw3000_c0_compute_rssi(struct dw3000 *dw, struct dw3000_rssi *rssi,
				  bool rx_tune, u8 sts)
{
	/* Details are logged into UWB-3455
	 * DW3700 User Manual v0.3 and 4 section 4.7.2 gives that RSSI
	 * can be computed using this formula:
	 * rssi = 10 * log10 ((cir_pwr * 2^21) / pacc_cnt ^ 2) + 6D - A(prf)
	 * But log10 isn't implemented ; let's use ilog2 instead, because it is
	 * easy to do on binary numbers. Thus, formula becomes:
	 * rssi = 3*log2(cir_pwr) - 6*log2(pacc_cnt) + 3*log2(2^21) + 6*D - A(prf)
	 * Notice that 3*log2(2^21) +6*D - A(prf) can be pre-computed.
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

const struct dw3000_chip_ops dw3000_chip_c0_ops = {
	.softreset = dw3000_c0_softreset,
	.init = dw3000_c0_init,
	.coex_init = dw3000_c0_coex_init,
	.coex_gpio = dw3000_c0_coex_gpio,
	.check_tx_ok = dw3000_c0_check_tx_ok,
	.prog_ldo_and_bias_tune = dw3000_c0_prog_ldo_and_bias_tune,
	.get_config_mrxlut_chan = dw3000_c0_get_config_mrxlut_chan,
	.get_dgc_dec = dw3000_c0_get_dgc_dec,
	.pre_read_sys_time = dw3000_c0_pre_read_sys_time,
	.pll_calibration_from_scratch = dw3000_c0_pll_calibration_from_scratch,
	.prog_pll_coarse_code = dw3000_c0_prog_pll_coarse_code,
	.set_mrxlut = dw3000_c0_set_mrxlut,
	.kick_ops_table_on_wakeup = dw3000_c0_kick_ops_table_on_wakeup,
	.kick_dgc_on_wakeup = dw3000_c0_kick_dgc_on_wakeup,
	.get_registers = dw3000_c0_get_registers,
	.compute_rssi = dw3000_c0_compute_rssi,
};
