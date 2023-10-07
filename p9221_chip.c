/* SPDX-License-Identifier: GPL-2.0 */
/*
 * P9221 Wireless Charger Driver chip revision specific functions.
 *
 * Copyright (C) 2020 Google, LLC
 *
 */

#include <linux/device.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/alarmtimer.h>
#include <misc/logbuffer.h>
#include "p9221_charger.h"
#include "google_bms.h"

#define P9XXX_NUM_GPIOS                 16
#define P9XXX_MIN_GPIO                  0
#define P9XXX_MAX_GPIO                  15
#define P9XXX_GPIO_CPOUT_EN             1
#define P9412_GPIO_CPOUT21_EN           2
#define P9XXX_GPIO_CPOUT_CTL_EN         3
#define P9XXX_GPIO_DC_SW_EN             4
#define P9XXX_GPIO_VBUS_EN              15

/* Simple Chip Specific Accessors */
/*
 * chip_get_rx_ilim
 *
 *   Get the receive current limit (mA).
 */
static int p9221_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = ((val * 100) + 200);
	return 0;
}

static int p9412_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = (val + 1) * 100;
	return 0;
}

static int p9222_chip_get_rx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9222RE_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = ((val * 100) + 100);
	return 0;
}

/*
 * chip_set_rx_ilim
 *
 *   Set the receive current limit (mA).
 */
static int p9221_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	/* uA -> 0.1A, offset 0.2A */
	if (ma < P9221_RX_ILIM_MIN_MA)
		return -EINVAL;

	if (ma > P9221_RX_ILIM_MAX_MA)
		ma = P9221_RX_ILIM_MAX_MA;

	val = (ma / 100) - 2;
	return chgr->reg_write_8(chgr, P9221R5_ILIM_SET_REG, val);
}

static int p9412_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	if (ma > P9412_RX_ILIM_MAX_MA)
		return -EINVAL;

	val = (ma / 100) - 1;
	return chgr->reg_write_8(chgr, P9221R5_ILIM_SET_REG, val);
}

static int p9222_chip_set_rx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	u8 val;

	if (ma < P9222_RX_ILIM_MIN_MA)
		return -EINVAL;

	if (ma > P9222_RX_ILIM_MAX_MA)
		ma = P9222_RX_ILIM_MAX_MA;

	val = (ma / 100) - 1;
	return chgr->reg_write_8(chgr, P9222RE_ILIM_SET_REG, val);
}

/*
 * chip_get_tx_ilim
 *
 *   Get the transmit current limit (mA).
 */
static int p9221_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	return -ENOTSUPP;
}

static int p9382_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9382A_ILIM_SET_REG, &val);
	if (ret)
		return ret;

	*ma = (u32) val;
	return 0;
}

static int p9412_chip_get_tx_ilim(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_TX_I_API_LIM_REG, &val);
	if (ret)
		return ret;

	*ma = (u32) val;
	return 0;
}

/*
 * chip_set_tx_ilim
 *
 *   Set the transmit current limit (mA).
 */
static int p9221_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	return -ENOTSUPP;
}

static int p9382_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	if ((ma < 0) || (ma > P9382A_RTX_ICL_MAX_MA))
		return -EINVAL;

	return chgr->reg_write_16(chgr, P9382A_ILIM_SET_REG, (u16) ma);
}

static int p9412_chip_set_tx_ilim(struct p9221_charger_data *chgr, u32 ma)
{
	if ((ma < 0) || (ma > P9382A_RTX_ICL_MAX_MA))
		return -EINVAL;

	return chgr->reg_write_16(chgr, P9412_TX_I_API_LIM_REG, (u16) ma);
}

/*
 * chip_get_die_temp
 *
 *   Get the chip temperature (milli-C).
 */
static int p9221_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_DIE_TEMP_ADC_REG, &val);
	if (ret)
		return ret;

	*mc = (val * 10 / 107 - 247) * 10;
	return 0;
}

static int p9412_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_DIE_TEMP_REG, &val);
	if (ret)
		return ret;

	*mc = P9221_C_TO_MILLIC(val);
	return 0;
}

static int p9222_chip_get_die_temp(struct p9221_charger_data *chgr, int *mc)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_DIE_TEMP_REG, &val);
	if (ret)
		return ret;

	*mc = P9221_C_TO_MILLIC(val);
	return 0;
}

/*
 * chip_get_iout
 *
 * get measure of current out (mA).
 */
static int p9xxx_chip_get_iout(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_IOUT_REG, &val);
	if (ret)
		return ret;

	*ma = val;
	return 0;
}

static int p9222_chip_get_iout(struct p9221_charger_data *chgr, u32 *ma)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_IOUT_REG, &val);
	if (ret)
		return ret;

	*ma = val;
	return 0;
}

/*
 * chip_get_vout
 *
 *   get voltage out (mV).
 */
static int p9xxx_chip_get_vout(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_VOUT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

static int p9222_chip_get_vout(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_VOUT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

/*
 * chip_get_vrect
 *
 *   Get rectified voltage out (mV).
 */
static int p9xxx_chip_get_vrect(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_VRECT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

static int p9222_chip_get_vrect(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_VRECT_REG, &val);
	if (ret)
		return ret;

	*mv = val;
	return 0;
}

/*
 * chip_get_op_freq
 *
 *   Get operating frequency (KHz).
 */
static int p9xxx_chip_get_op_freq(struct p9221_charger_data *chgr, u32 *khz)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9221R5_OP_FREQ_REG, &val);
	if (ret)
		return ret;

	*khz = (u32) val;
	return 0;
}

static int p9222_chip_get_op_freq(struct p9221_charger_data *chgr, u32 *khz)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9222RE_OP_FREQ_REG, &val);
	if (ret)
		return ret;

	*khz = (u32) val;
	return 0;
}
/*
 * chip_get_vcpout
 *
 *   Get CPout voltage(mV)
 */
static int p9412_chip_get_vcpout(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_VCPOUT_VOL_REG, &val);
	if (ret)
		return ret;

	*mv = (u32) val;
	return 0;
}
static int p9xxx_chip_get_vcpout(struct p9221_charger_data *chgr, u32 *mv)
{
	return -ENOTSUPP;
}


/*
 * chip_get_vout_max
 *
 *   Get the maximum output voltage (mV).
 */
static int p9221_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	return -ENOTSUPP;
}

static int p9832_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9221R5_VOUT_SET_REG, &val);
	if (ret)
		return ret;

	*mv = val * 100; /* 100 mV units */
	return 0;
}

static int p9412_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u16 val;

	ret = chgr->reg_read_16(chgr, P9412_VOUT_SET_REG, &val);
	if (ret)
		return ret;

	*mv = val * 10; /* 10 mV units */
	return 0;
}

static int p9222_chip_get_vout_max(struct p9221_charger_data *chgr, u32 *mv)
{
	int ret;
	u8 val;

	ret = chgr->reg_read_8(chgr, P9222RE_VOUT_SET_REG, &val);
	if (ret)
		return ret;

	*mv = val * 100 + 3500; /* Vout = value * 100mV + 3500mV */
	return 0;
}

/*
 * chip_set_vout_max
 *
 *   Set the maximum output voltage (mV).
 */
static int p9221_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	return -ENOTSUPP;
}

static int p9832_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	int ret;

	if (mv < P9221_VOUT_SET_MIN_MV || mv > chgr->pdata->max_vout_mv)
		return -EINVAL;

	ret = chgr->reg_write_8(chgr, P9221R5_VOUT_SET_REG, mv / 100);
	return ret;
}

static int p9412_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	int ret;

	if (mv < P9412_VOUT_SET_MIN_MV || mv > chgr->pdata->max_vout_mv)
		return -EINVAL;

	ret = chgr->reg_write_16(chgr, P9412_VOUT_SET_REG, mv / 10);
	return ret;
}

static int p9222_chip_set_vout_max(struct p9221_charger_data *chgr, u32 mv)
{
	int ret;

	if (mv < P9222_VOUT_SET_MIN_MV || mv > P9222_VOUT_SET_MAX_MV)
		return -EINVAL;

	ret = chgr->reg_write_8(chgr, P9222RE_VOUT_SET_REG, (mv - 3500) / 100);
	return ret;
}

/* system mode register */
static int p9221_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	int ret;
	u8 val8;

	ret = chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG, &val8);
	if (ret)
		return ret;

	/* map to p9412 values */
	if (val8 & P9221R5_MODE_WPCMODE)
		*mode = P9XXX_SYS_OP_MODE_WPC_BASIC;
	else if (val8 & P9382A_MODE_TXMODE)
		*mode = P9XXX_SYS_OP_MODE_TX_MODE;
	else if (val8 & P9221R5_MODE_EXTENDED)
		*mode = P9XXX_SYS_OP_MODE_WPC_EXTD;
	else
		*mode = P9XXX_SYS_OP_MODE_AC_MISSING;
	return 0;
}

static int p9222_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	int ret;
	u8 val8;

	ret = chgr->reg_read_8(chgr, P9222RE_SYSTEM_MODE_REG, &val8);
	if (ret)
		return ret;

	/* map to p9412 values */
	if (val8 & P9222_SYS_OP_MODE_WPC_BASIC)
		*mode = P9XXX_SYS_OP_MODE_WPC_BASIC;
	else if (val8 & P9222_SYS_OP_MODE_WPC_EXTD)
		*mode = P9XXX_SYS_OP_MODE_WPC_EXTD;
	else
		*mode = P9XXX_SYS_OP_MODE_AC_MISSING;

	return 0;
}

static int p9412_chip_get_sys_mode(struct p9221_charger_data *chgr, u8 *mode)
{
	return chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG, mode);
}

/* These are more involved than just chip access */

static int p9382_wait_for_mode(struct p9221_charger_data *chgr, int mode)
{
	int loops, ret;
	uint8_t sys_mode;

	/* 20 * 50 = 1 sec */
	for (loops = 20 ; loops ; loops--) {
		ret = chgr->reg_read_8(chgr, P9221R5_SYSTEM_MODE_REG,
					&sys_mode);
		if (ret < 0) {
			dev_err(&chgr->client->dev,
				"cannot read system_mode (%d)", ret);
			return -EIO;
		}

		if (sys_mode == mode)
			return 0;

		msleep(50);
	}

	return -ETIMEDOUT;
}

/* get the data buf for receive */
static int p9221_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_RECV_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9221R5_DATA_RECV_BUF_START, data, len);
}

static int p9222_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9222RE_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9222RE_DATA_BUF_START, data, len);
}

static int p9382_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_RECV_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9382A_DATA_RECV_BUF_START, data, len);
}

static int p9412_get_data_buf(struct p9221_charger_data *chgr,
			      u8 data[], size_t len)
{
	if (!len || len > P9412_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_read_n(chgr, P9412_DATA_BUF_START, data, len);
}

/* set the data buf for send */
static int p9221_set_data_buf(struct p9221_charger_data *chgr,
			      const u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9221R5_DATA_SEND_BUF_START, data, len);
}

static int p9222_set_data_buf(struct p9221_charger_data *chgr,
			      const u8 data[], size_t len)
{
	if (!len || len > P9222RE_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9222RE_DATA_BUF_START, data, len);
}

static int p9382_set_data_buf(struct p9221_charger_data *chgr,
			      const u8 data[], size_t len)
{
	if (!len || len > P9221R5_DATA_SEND_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9382A_DATA_SEND_BUF_START, data, len);
}

static int p9412_set_data_buf(struct p9221_charger_data *chgr,
				   const u8 data[], size_t len)
{
	if (!len || len > P9412_DATA_BUF_SIZE)
		return -EINVAL;

	return chgr->reg_write_n(chgr, P9412_DATA_BUF_START, data, len);
}

/* receive size */
static int p9221_get_cc_recv_size(struct p9221_charger_data *chgr, size_t *len)
{
	int ret;
	u8 len8;

	ret = chgr->reg_read_8(chgr, P9221R5_COM_CHAN_RECV_SIZE_REG, &len8);
	if (!ret)
		*len = len8;
	return ret;
}

static int p9222_get_cc_recv_size(struct p9221_charger_data *chgr, size_t *len)
{
	int ret;
	u8 len8;

	ret = chgr->reg_read_8(chgr, P9222RE_COM_CHAN_RECV_SIZE_REG, &len8);
	if (ret == 0)
		*len = len8;
	return ret;
}

static int p9412_get_cc_recv_size(struct p9221_charger_data *chgr, size_t *len)
{
	int ret;
	u16 len16;

	ret = chgr->reg_read_16(chgr, P9412_COM_CHAN_RECV_SIZE_REG, &len16);
	if (!ret)
		*len = len16;
	return ret;
}

/* send size */
static int p9221_set_cc_send_size(struct p9221_charger_data *chgr, size_t len)
{
	return chgr->reg_write_8(chgr, P9221R5_COM_CHAN_SEND_SIZE_REG, len);
}

static int p9222_set_cc_send_size(struct p9221_charger_data *chgr, size_t len)
{
	int ret;

	/* set packet type(BiDi) */
	ret = chgr->reg_write_8(chgr, chgr->reg_packet_type_addr,
				BIDI_COM_PACKET_TYPE);
	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to write packet type %d\n", ret);
		return ret;
	}

	return chgr->reg_write_16(chgr, P9222RE_COM_CHAN_SEND_SIZE_REG, len);
}

static int p9382_set_cc_send_size(struct p9221_charger_data *chgr, size_t len)
{
	int ret;

	/* set packet type to 0x100 */
	ret = chgr->reg_write_8(chgr, P9382A_COM_PACKET_TYPE_ADDR,
				BIDI_COM_PACKET_TYPE);
	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to write packet type %d\n", ret);
		return ret;
	}

	return chgr->reg_write_16(chgr, P9382A_COM_CHAN_SEND_SIZE_REG, len);
}

static int p9412_set_cc_send_size(struct p9221_charger_data *chgr, size_t len)
{
	int ret;

	/* set as ADT type(Authentication) */
	if (chgr->auth_type != 0) {
		ret = chgr->reg_write_8(chgr, P9412_COM_PACKET_TYPE_ADDR,
					(P9412_ADT_TYPE_AUTH << 3) |
					(chgr->tx_len >> 8));
		ret |= chgr->reg_write_8(chgr, P9412_COM_PACKET_TYPE_ADDR + 1,
					 chgr->tx_len & 0xFF);
		chgr->auth_type = 0;
	} else {
		/* set packet type(BiDi 0x98) to 0x800 */
		ret = chgr->reg_write_8(chgr, P9412_COM_PACKET_TYPE_ADDR,
					BIDI_COM_PACKET_TYPE);
	}

	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to write packet type %d\n", ret);
		return ret;
	}

	return chgr->reg_write_16(chgr, P9412_COM_CHAN_SEND_SIZE_REG, len);
}

/* get align x */
static int p9221_get_align_x(struct p9221_charger_data *chgr, u8 *x)
{
	return chgr->reg_read_8(chgr, P9221R5_ALIGN_X_ADC_REG, x);
}

static int p9412_get_align_x(struct p9221_charger_data *chgr, u8 *x)
{
	return chgr->reg_read_8(chgr, P9412_ALIGN_X_REG, x);
}

/* get align y */
static int p9221_get_align_y(struct p9221_charger_data *chgr, u8 *y)
{
	return chgr->reg_read_8(chgr, P9221R5_ALIGN_Y_ADC_REG, y);
}

static int p9412_get_align_y(struct p9221_charger_data *chgr, u8 *y)
{
	return chgr->reg_read_8(chgr, P9412_ALIGN_Y_REG, y);
}

/* Simple Chip Specific Logic Functions */

/* Disable/Enable transmit mode. */
static int p9221_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);
	return -ENOTSUPP;
}

static int p9382_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	int ret;
	u16 rev;

	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);
	if (enable) {
		/* check FW revision */
		ret = chgr->reg_read_16(chgr,
					P9221_OTP_FW_MINOR_REV_REG, &rev);
		if (ret)
			return ret;

		if (rev >= P9382A_FW_REV_25) {
			/* write 0x0003 to 0x69 after rev 25 */
			ret = chgr->reg_write_16(chgr,
						 P9382A_TRX_ENABLE_REG,
						 P9382A_TX_INHIBIT);
		} else {
			/* write 0x0000 to 0x34 */
			ret = chgr->reg_write_16(chgr,
						 P9382A_STATUS_REG, 0);
		}

		/* check 0x4C reads back as 0x04 */
		ret = p9382_wait_for_mode(chgr, P9382A_MODE_TXMODE);
	} else {
		/* Write 0x80 to 0x4E, check 0x4C reads back as 0x0000 */
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_RENEGOTIATE);
		if (ret == 0) {
			ret = p9382_wait_for_mode(chgr, 0);
			if (ret < 0)
				pr_err("cannot exit rTX mode (%d)\n", ret);
		}
	}

	return ret;
}

static int p9412_chip_tx_mode(struct p9221_charger_data *chgr, bool enable)
{
	int ret;

	logbuffer_log(chgr->rtx_log, "%s(%d)", __func__, enable);

	if (enable) {
		if (chgr->pdata->apbst_en) {
			ret = chgr->reg_write_8(chgr, P9412_APBSTPING_REG, 0);
			ret |= chgr->reg_write_8(chgr, P9412_APBSTCONTROL_REG, P9412_APBSTPING_7V);
			logbuffer_log(chgr->rtx_log,
				"configure Ext-Boost Vout to 5V.(%d)", ret);
			if (ret < 0)
				return ret;
		}
		ret = chgr->reg_write_8(chgr, P9412_TX_CMD_REG, P9412_TX_CMD_TX_MODE_EN);
		if (ret) {
			logbuffer_log(chgr->rtx_log,
				 "tx_cmd_reg write failed (%d)", ret);
			return ret;
		}
		ret = p9382_wait_for_mode(chgr, P9XXX_SYS_OP_MODE_TX_MODE);
		if (ret) {
			logbuffer_log(chgr->rtx_log,
				      "error waiting for tx_mode (%d)", ret);
			return ret;
		}

		ret = chgr->reg_write_16(chgr, P9412_TXOCP_REG, P9412_TXOCP_1400MA);
		logbuffer_log(chgr->rtx_log, "configure TX OCP to %dMA", P9412_TXOCP_1400MA);
		if (ret < 0)
			return ret;

		if (!chgr->pdata->apbst_en)
			return ret;
		mod_delayed_work(system_wq, &chgr->chk_rtx_ocp_work, 0);
	} else {
		ret = chgr->chip_set_cmd(chgr, P9412_CMD_TXMODE_EXIT);
		if (ret == 0) {
			ret = p9382_wait_for_mode(chgr, 0);
			if (ret < 0)
				pr_err("cannot exit rTX mode (%d)", ret);
		}
		if (chgr->pdata->apbst_en) {
			ret = chgr->reg_write_8(chgr, P9412_APBSTPING_REG, 0);
			logbuffer_log(chgr->rtx_log,
				"configure Ext-Boost back to 5V.(%d)", ret);
		}
	}

	return ret;
}

static int p9222_chip_set_cmd_reg(struct p9221_charger_data *chgr, u16 cmd)
{
	u16 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = chgr->reg_read_16(chgr, P9222RE_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&chgr->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = chgr->reg_write_16(chgr, P9222RE_COM_REG, (u16)cmd);
	if (ret)
		dev_err(&chgr->client->dev,
			"Failed to set cmd reg %02x: %d\n", (u16)cmd, ret);

	return ret;
}

static int p9xxx_chip_set_cmd_reg(struct p9221_charger_data *chgr, u16 cmd)
{
	u16 cur_cmd = 0;
	int retry;
	int ret;

	for (retry = 0; retry < P9221_COM_CHAN_RETRIES; retry++) {
		ret = chgr->reg_read_16(chgr, P9221_COM_REG, &cur_cmd);
		if (ret == 0 && cur_cmd == 0)
			break;
		msleep(25);
	}

	if (retry >= P9221_COM_CHAN_RETRIES) {
		dev_err(&chgr->client->dev,
			"Failed to wait for cmd free %02x\n", cur_cmd);
		return -EBUSY;
	}

	ret = chgr->reg_write_16(chgr, P9221_COM_REG, cmd);
	if (ret)
		dev_err(&chgr->client->dev,
			"Failed to set cmd reg %02x: %d\n", cmd, ret);

	return ret;
}

/* ccreset */
static int p9221_send_ccreset(struct p9221_charger_data *chgr)
{
	int ret;

	dev_info(&chgr->client->dev, "%s CC reset\n",
		 chgr->is_mfg_google? "Send" : "Ignore");

	if (chgr->is_mfg_google == false)
		return 0;

	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_8(chgr, P9221R5_COM_CHAN_RESET_REG,
				P9221R5_COM_CHAN_CCRESET);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_CCACTIVATE);

	mutex_unlock(&chgr->cmd_lock);
	return ret;
}

static int p9412_send_ccreset(struct p9221_charger_data *chgr)
{
	int ret = 0;

	dev_info(&chgr->client->dev, "%s CC reset\n",
		 chgr->is_mfg_google? "Send" : "Ignore");

	if (chgr->is_mfg_google == false)
		return 0;

	mutex_lock(&chgr->cmd_lock);

	if (chgr->cc_reset_pending) {
		mutex_unlock(&chgr->cmd_lock);
		goto error_done;
	}

	chgr->cc_reset_pending = true;

	ret = chgr->reg_write_8(chgr, P9412_COM_PACKET_TYPE_ADDR,
				CHANNEL_RESET_PACKET_TYPE);
	ret |= chgr->reg_write_8(chgr, P9412_COM_CHAN_SEND_SIZE_REG, 0);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9412_COM_CCACTIVATE);

	mutex_unlock(&chgr->cmd_lock);

	if (ret < 0) {
		chgr->cc_reset_pending = false;
		goto error_done;
	}

	/* ccreset needs 100ms, so set the timeout to 500ms */
	wait_event_timeout(chgr->ccreset_wq, chgr->cc_reset_pending == false,
			   msecs_to_jiffies(500));

	if (chgr->cc_reset_pending) {
		/* TODO: offline or command failed? */
		chgr->cc_reset_pending = false;
		ret = -ETIMEDOUT;
	}

error_done:
	if (ret != 0)
		dev_err(&chgr->client->dev, "Error sending CC reset (%d)\n",
			ret);
	return ret;
}

/* send eop */
/*   send multiple times to make sure it works */
#define EOP_RESTART_COUNT	2
#define P9222_EOP_REPEAT_COUNT	4
static int p9222_send_repeat_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int count, ret = 0;
	u8 val, ept_reason;

	for (count = 0; count < P9222_EOP_REPEAT_COUNT; count++) {
		ept_reason = (count < EOP_RESTART_COUNT) ? P9221_EOP_RESTART_POWER : reason;

		ret = chgr->reg_write_8(chgr, P9222RE_EPT_REG, ept_reason);
		if (ret == 0)
			ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);
		if (ret < 0) {
			dev_err(&chgr->client->dev, "fail send eop_%d (%d)\n", count, ret);
			return ret;
		}
		mdelay(500);

		/* Check Tx is offline due to EPT command works */
		ret = chgr->reg_read_8(chgr, P9221_STATUS_REG, &val);
		if (ret < 0) {
			dev_info(&chgr->client->dev,
				 "WLC chip offline, count=%d, ret=%d\n", count, ret);
			break;
		}
	}

	dev_info(&chgr->client->dev,
		 "send 3xEOP command success(reason=%02x)\n", reason);

	return 0;
}

static int p9221_send_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int ret;

	dev_info(&chgr->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_8(chgr, P9221R5_EPT_REG, reason);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);

	mutex_unlock(&chgr->cmd_lock);
	return ret;
}
/* send eop */
static int p9222_send_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int ret;

	dev_info(&chgr->client->dev, "Send P9222 EOP reason=%d\n", reason);

	mutex_lock(&chgr->cmd_lock);

	if (chgr->pdata->disable_repeat_eop) {
		ret = chgr->reg_write_8(chgr, P9222RE_EPT_REG, reason);
		if (ret == 0)
			ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);
	} else {
		ret = p9222_send_repeat_eop(chgr, reason);
	}

	mutex_unlock(&chgr->cmd_lock);

	return ret;
}

/* 3 times to make sure it works */
static int p9412_send_3eop(struct p9221_charger_data *chgr, u8 reason)
{
	int count, ret = 0;
	u8 val;

	for (count = 0; count < 3; count++) {
		/* Command P9412 to send EPT */
		ret = chgr->reg_write_8(chgr, P9221R5_EPT_REG, reason);
		if (ret == 0)
			ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);
		if (ret < 0) {
			dev_err(&chgr->client->dev, "fail send eop%d (%d)\n",
				count, ret);
			return ret;
		} else
			dev_info(&chgr->client->dev, "send eop command success\n");

		mdelay(500);

		ret = chgr->reg_read_8(chgr, P9221_STATUS_REG, &val);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int p9412_send_eop(struct p9221_charger_data *chgr, u8 reason)
{
	int ret = 0;

	dev_info(&chgr->client->dev, "Send EOP reason=%d\n", reason);

	mutex_lock(&chgr->cmd_lock);

	if (reason == P9221_EOP_REVERT_TO_BPP) {
		ret = chgr->reg_write_8(chgr, P9221R5_EPT_REG, reason);
		if (ret == 0)
			ret = chgr->chip_set_cmd(chgr, P9221R5_COM_SENDEPT);
	} else {
		ret = p9412_send_3eop(chgr, reason);
	}

	mutex_unlock(&chgr->cmd_lock);

	return ret;
}

/* RTx mode: send Tx ID */
static int p9221_send_txid(struct p9221_charger_data *chgr)
{
	return -ENOTSUPP;
}

static int p9xxx_send_txid(struct p9221_charger_data *chgr)
{
	int ret;

	mutex_lock(&chgr->cmd_lock);

	/* write packet type to 0x100 */
	ret = chgr->reg_write_8(chgr, chgr->reg_packet_type_addr,
				PROPRIETARY_PACKET_TYPE);
	if (ret)
		goto error;

	memset(chgr->tx_buf, 0, chgr->tx_buf_size);

	/* write 0x4F as header to 0x104 */
	chgr->tx_buf[0] = FAST_SERIAL_ID_HEADER;
	chgr->tx_len = FAST_SERIAL_ID_SIZE;

	/* TODO: write txid to bit(23, 0) */
	memset(&chgr->tx_buf[1], 0x12, FAST_SERIAL_ID_SIZE - 1);

        /* write phone type to bit(23, 17) */
        chgr->tx_buf[3] = chgr->pdata->phone_type << 1;

	/* write accessory type to bit(31, 24) */
	chgr->tx_buf[4] = TX_ACCESSORY_TYPE;

	ret |= p9xxx_chip_set_pp_buf(chgr, chgr->tx_buf, chgr->tx_len + 1);
	if (ret) {
		dev_err(&chgr->client->dev, "Failed to load tx %d\n", ret);
		goto error;
	}

	/* send packet */
	ret = chgr->chip_set_cmd(chgr, P9221R5_COM_CCACTIVATE);
	if (ret)
		dev_err(&chgr->client->dev, "Failed to send txid %d\n", ret);
	else
		chgr->last_capacity = -1;

error:
	mutex_unlock(&chgr->cmd_lock);
	return ret;
}

/* RTx mode: send csp in Tx mode */
static int p9221_send_csp_in_txmode(struct p9221_charger_data *chgr, u8 stat)
{
	return -ENOTSUPP;
}
static int p9xxx_send_csp_in_txmode(struct p9221_charger_data *chgr, u8 stat)
{
	int ret = 0;
	u16 status_reg;

	ret = chgr->reg_read_16(chgr, P9221_STATUS_REG, &status_reg);
	if ((ret != 0) || (status_reg & chgr->ints.rx_connected_bit) == 0)
		return -EINVAL;

	dev_info(&chgr->client->dev, "Send Tx soc=%d\n", stat);
	/* write packet type to 0x100 */
	ret = chgr->reg_write_8(chgr,
				chgr->reg_packet_type_addr,
				PROPRIETARY_PACKET_TYPE);

	if (ret != 0)
		return ret;

	memset(chgr->tx_buf, 0, P9221R5_DATA_SEND_BUF_SIZE);

	chgr->tx_len = CHARGE_STATUS_PACKET_SIZE;
	/* write 0x48 to 0x104 */
	chgr->tx_buf[0] = CHARGE_STATUS_PACKET_HEADER;
	/* sype: power control 0x08 */
	chgr->tx_buf[1] = PP_TYPE_POWER_CONTROL;
	/* subtype: state of charge report 0x10 */
	chgr->tx_buf[2] = PP_SUBTYPE_SOC;
	/* soc: allow for 0.5% to fill up 0-200 -> 0-100% */
	chgr->tx_buf[3] = stat * 2;
	/* crc-8 */
	chgr->tx_buf[4] = p9221_crc8(&chgr->tx_buf[1], chgr->tx_len - 1,
				     CRC8_INIT_VALUE);

	ret = p9xxx_chip_set_pp_buf(chgr, chgr->tx_buf, chgr->tx_len + 1);

	ret |= chgr->chip_set_cmd(chgr, P9221R5_COM_CCACTIVATE);

	memset(chgr->tx_buf, 0, P9221R5_DATA_SEND_BUF_SIZE);
	chgr->tx_len = 0;

	return ret;
}

/* renegotiate power from charger->pdata->epp_rp_value */
static int p9221_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret;
	int val8 = P9412_MW_TO_HW(chgr->pdata->epp_rp_value);

	/* units 0.5 W*/
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_NEGOTIATED_POWER_REG, val8);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to EPP_NEG_POWER=%d (%d)\n",
			val8, ret);
	return ret;
}

static int p9222_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret = 0, guar_pwr_mw, cnt;
	u8 val8, rp8;

	if (chgr->check_rp != RP_CHECKING)
		return ret;

	ret = chgr->reg_read_8(chgr, P9222RE_EPP_TX_GUARANTEED_POWER_REG, &val8);
	if (ret)
		return ret;
	guar_pwr_mw = P9412_HW_TO_MW(val8);

	/* write renegotiated power to 11W(>10W) or 10W(<=10W) */
	if (chgr->pdata->epp_rp_low_value != -1 && guar_pwr_mw <= P9222_NEG_POWER_10W)
		rp8 = P9412_MW_TO_HW(chgr->pdata->epp_rp_low_value);
	else
		rp8 = P9412_MW_TO_HW(chgr->pdata->epp_rp_value);

	/*
	 * The neg_pwr write window is 200ms to 340ms, write every 20ms to make
	 * sure it works
	 */
	for (cnt = 0; cnt < 7 ; cnt++) {
		/* units 0.5 W */
		ret = chgr->reg_write_8(chgr, P9222RE_EPP_REQ_NEGOTIATED_POWER_REG, rp8);
		ret |= chgr->reg_write_8(chgr, P9222RE_EPP_REQ_MAXIMUM_POWER_REG, rp8);

		usleep_range(20 * USEC_PER_MSEC, 22 * USEC_PER_MSEC);
		if (!chgr->online)
			return -ENODEV;
	}
	if (ret == 0)
		logbuffer_log(chgr->log, "read neg_pwr=0x%x, write neg_pwr=0x%x(guar_pwr=%dW)",
			      val8, rp8, guar_pwr_mw/1000);

	chgr->check_rp = RP_DONE;

	return ret;
}

static int p9412_chip_renegotiate_pwr(struct p9221_charger_data *chgr)
{
	int ret;
	u8 val8;
	int try;
	int guar_pwr_mw; /* power provided by charger */
	int cur_pwr_mw;  /* power currently supplied */
	int tgt_pwr_mw;  /* power to be requested */
	int cfg_pwr_mw = chgr->pdata->epp_rp_value;

	ret = chgr->chip_get_sys_mode(chgr, &val8);
	if (ret)
		goto out;

	if (val8 != P9XXX_SYS_OP_MODE_WPC_EXTD)
		return 0;

	logbuffer_log(chgr->log, "%s: WPC renegotiation", __func__);

	/*
	 * Compare the current power to the available power o
	 * determine if renegotiation is worthwhile.
	 */
	ret = chgr->reg_read_8(chgr, P9221R5_EPP_TX_GUARANTEED_POWER_REG,
			       &val8);
	if (ret)
		goto out;
	guar_pwr_mw = P9412_HW_TO_MW(val8);

	ret = chgr->reg_read_8(chgr, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG,
			       &val8);
	if (ret)
		goto out;
	cur_pwr_mw = P9412_HW_TO_MW(val8);

	tgt_pwr_mw = min(cfg_pwr_mw, guar_pwr_mw);
	logbuffer_log(chgr->log, "%s: tgt pwr = %d cur pwr = %d mW",
		      __func__, tgt_pwr_mw, cur_pwr_mw);

	if (cur_pwr_mw >= tgt_pwr_mw) {
		ret = -EAGAIN;
		logbuffer_log(chgr->log, "%s: no extra power available",
			      __func__);
		goto out;
	}

	/* Set the voltage to maximum value as defined in device-tree. */
	ret = chgr->chip_set_vout_max(chgr, chgr->pdata->epp_vout_mv);

	val8 = P9412_MW_TO_HW(tgt_pwr_mw);
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_NEGOTIATED_POWER_REG,
				val8);
	if (ret) {
		dev_err(&chgr->client->dev,
			"cannot write to EPP_NEG_POWER=%d (%d)\n", val8, ret);
		goto out;
	}

	val8 = P9412_MW_TO_HW(tgt_pwr_mw);
	ret = chgr->reg_write_8(chgr,
				P9221R5_EPP_REQ_MAXIMUM_POWER_REG,
				val8);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to EPP_MAX_POWER=%d (%d)\n",
			 chgr->pdata->epp_rp_value, ret);

	ret = chgr->chip_set_cmd(chgr, P9221_COM_RENEGOTIATE);
	if (ret < 0)
		dev_err(&chgr->client->dev,
			"cannot write to sys_cmd =%d (%d)\n",
			 chgr->pdata->epp_rp_value, ret);

	/* Wait for renegotiation to complete. */
	ret = -ETIME;
	for (try = 0; try < P9412_RN_MAX_POLL_ATTEMPTS; try++) {
		msleep(P9412_RN_DELAY_MS);
		ret = chgr->reg_read_8(chgr,
				       P9221R5_EPP_RENEGOTIATION_REG,
				       &val8);
		if (val8 & P9412_RN_STATUS_DONE) {
			ret = 0;
			break;
		} else if (val8 & P9412_RN_STATUS_ERROR) {
			ret = -EIO;
			break;
		}
	}
	logbuffer_log(chgr->log, "%s: status = 0x%02x (tries = %d)",
		      __func__, val8, try);
out:
	return ret;
}
/* Read EPP_CUR_NEGOTIATED_POWER_REG to configure DC_ICL for EPP */
static void p9xxx_check_neg_power(struct p9221_charger_data *chgr)
{
	int ret;
	u8 np8;

	chgr->dc_icl_epp_neg = P9221_DC_ICL_EPP_UA;

	if (chgr->chip_id < P9382A_CHIP_ID)
		return;

	if (chgr->is_mfg_google) {
		chgr->dc_icl_epp_neg = P9XXX_DC_ICL_EPP_1000;
		dev_info(&chgr->client->dev,
			 "mfg code=%02x, use dc_icl=%dmA\n",
			 WLC_MFG_GOOGLE, P9XXX_DC_ICL_EPP_1000);
		return;
	}
	ret = chgr->reg_read_8(chgr, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG, &np8);
	if (ret)
		dev_err(&chgr->client->dev,
			"Could not read Tx neg power: %d\n", ret);
	else if (np8 < P9XXX_NEG_POWER_10W) {
		/*
		 * base on firmware 17
		 * Vout is 5V when Tx<10W, use BPP ICL
		 */
		chgr->dc_icl_epp_neg = P9221_DC_ICL_BPP_UA;
		dev_info(&chgr->client->dev,
			 "EPP less than 10W,use dc_icl=%dmA,np=%02x\n",
			 P9221_DC_ICL_BPP_UA/1000, np8);
	} else if (np8 < P9XXX_NEG_POWER_11W) {
		chgr->dc_icl_epp_neg = P9XXX_DC_ICL_EPP_1000;
		dev_info(&chgr->client->dev,
			 "Use dc_icl=%dmA,np=%02x\n",
			 chgr->dc_icl_epp_neg/1000, np8);
	}
}

static void p9222_check_neg_power(struct p9221_charger_data *chgr)
{

}

static int p9221_capdiv_en(struct p9221_charger_data *chgr, u8 mode)
{
	return -ENOTSUPP;
}

static int p9412_capdiv_en(struct p9221_charger_data *chgr, u8 mode)
{
	const u8 mask = mode;
	int ret, loops;
	u8 cdmode;

	/* TODO: it probably needs a lock around this */

	ret = chgr->reg_read_8(chgr, P9412_CDMODE_STS_REG, &cdmode);
	if (ret < 0)
		return ret;

	/* If the results are as expected, no changes needed */
	if ((cdmode & mask) == mask)
		return ret;

	ret = chgr->reg_write_8(chgr, P9412_CDMODE_REQ_REG, mask);
	if (ret < 0)
		return -EIO;

	ret = chgr->chip_set_cmd(chgr, INIT_CAP_DIV_CMD);
	if (ret < 0)
		return -EIO;

	msleep(50);

	/* verify the change to Cap Divider mode */
	for (loops = 30 ; loops ; loops--) {

		ret = chgr->reg_read_8(chgr, P9412_CDMODE_STS_REG, &cdmode);
		if (ret < 0)
			return ret;

		if ((cdmode & mask) == mask)
			break;

		msleep(100);
	}

	return ((cdmode & mask) == mask) ? 0 :  -ETIMEDOUT;
}


static int p9221_prop_mode_enable(struct p9221_charger_data *chgr, int req_pwr)
{
	return -ENOTSUPP;
}

#define MAX77759_CHGR_MODE_ALL_OFF		0
/* b/202795383 remove load current before enable P9412 CD mode */
static int p9412_prop_mode_capdiv_enable(struct p9221_charger_data *chgr)
{
	int ret, i;


	/* TODO: need to become a fake GPIO in the max77759 charger */
	if (!chgr->chg_mode_votable)
		chgr->chg_mode_votable =
			gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (chgr->chg_mode_votable)
		gvotable_cast_long_vote(chgr->chg_mode_votable,
					P9221_WLC_VOTER,
					MAX77759_CHGR_MODE_ALL_OFF, true);

	/* total 2 seconds wait and early exit when WLC offline */
	for (i = 0; i < 20; i += 1) {
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
		if (!chgr->online) {
			ret = -ENODEV;
			goto error_exit;
		}
	}

	ret = p9412_capdiv_en(chgr, CDMODE_CAP_DIV_MODE);

	/* total 1 seconds wait and early exit when WLC offline */
	for (i = 0; i < 10; i += 1) {
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
		if (!chgr->online) {
			ret = -ENODEV;
			goto error_exit;
		}
	}

error_exit:
	if (chgr->chg_mode_votable)
		gvotable_cast_long_vote(chgr->chg_mode_votable,
					P9221_WLC_VOTER,
					MAX77759_CHGR_MODE_ALL_OFF, false);

	return ret;
}

static int p9412_prop_mode_enable(struct p9221_charger_data *chgr, int req_pwr)
{
	int ret, loops, i;
	u8 val8, cdmode, txpwr, pwr_stp, mode_sts, err_sts, prop_cur_pwr, prop_req_pwr;
	u32 val = 0;

	if (p9xxx_is_capdiv_en(chgr))
		goto err_exit;

	ret = chgr->chip_get_sys_mode(chgr, &val8);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: cannot get sys mode\n");
		return 0;
	}

	if (val8 == P9XXX_SYS_OP_MODE_PROPRIETARY) {

		ret = chgr->reg_read_8(chgr, P9412_PROP_TX_POTEN_PWR_REG, &txpwr);
		txpwr = txpwr / 2;
		if (ret != 0 || txpwr < HPP_MODE_PWR_REQUIRE) {
			dev_info(&chgr->client->dev,
				"PROP_MODE: power=%dW not supported\n", txpwr);
			chgr->prop_mode_en = false;
			goto err_exit;
		}

		goto enable_capdiv;
	}

	ret = p9xxx_chip_get_tx_mfg_code(chgr, &chgr->mfg);
	if (chgr->mfg != WLC_MFG_GOOGLE) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: mfg code =%02x\n", chgr->mfg);
		return 0;
	}

	/*
	 * Step 0: clear data type buffer:
	 * write 0 to 0x800 and 0x801
	 */
	ret = chgr->reg_write_n(chgr,
				P9412_COM_PACKET_TYPE_ADDR,
				&val, 4);
	if (ret) {
		dev_err(&chgr->client->dev,
			"Failed to clear data type buffer: %d\n", ret);
		goto err_exit;
	}

	/*
	 * Step 1: clear all interrupts:
	 * write 0xFFFF to 0x3A then write 0x20 to 0x4E
	 */
	mutex_lock(&chgr->cmd_lock);

	ret = chgr->reg_write_16(chgr,
				 P9221R5_INT_CLEAR_REG, P9XXX_INT_CLEAR_MASK);
	if (ret == 0)
		ret = chgr->chip_set_cmd(chgr, P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&chgr->client->dev, "Failed to reset INT: %d\n", ret);
		mutex_unlock(&chgr->cmd_lock);
		goto err_exit;
	}

	mutex_unlock(&chgr->cmd_lock);

	msleep(50);

enable_capdiv:
	if (chgr->pdata->has_sw_ramp) {
		dev_dbg(&chgr->client->dev, "%s: voter=%s, icl=%d\n",
			__func__, P9221_HPP_VOTER, P9XXX_CDMODE_ENABLE_ICL_UA);
		ret = p9xxx_sw_ramp_icl(chgr, P9XXX_CDMODE_ENABLE_ICL_UA);
		if (ret < 0)
			return ret;

		ret = gvotable_cast_int_vote(chgr->dc_icl_votable, P9221_HPP_VOTER,
					     P9XXX_CDMODE_ENABLE_ICL_UA, true);
		if (ret == 0)
			ret = gvotable_cast_int_vote(chgr->dc_icl_votable,
						     P9221_RAMP_VOTER, 0, false);
		if (ret < 0)
			dev_err(&chgr->client->dev, "%s: cannot setup sw ramp (%d)\n",
				__func__, ret);

	}

	/*
	 * Step 2: enable Cap Divider configuration:
	 * write 0x02 to 0x101 then write 0x40 to 0x4E
	 */
	ret = p9412_prop_mode_capdiv_enable(chgr);
	if (ret) {
		dev_err(&chgr->client->dev, "PROP_MODE: fail to enable Cap Div mode\n");
		goto err_exit;
	}

	for (i = 0; i < 30; i += 1) {
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
		if (!chgr->online)
			goto err_exit;
	}

	/*
	 * Step 3: Enable Proprietary Mode: write 0x01 to 0x4F (0x4E bit8)
	 */
	if (val8 == P9XXX_SYS_OP_MODE_PROPRIETARY)
		goto request_pwr;

	ret = chgr->chip_set_cmd(chgr, PROP_MODE_EN_CMD);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to send PROP_MODE_EN_CMD\n");
		goto err_exit;
	}

	msleep(50);

	/*
	 * Step 4: wait for PropModeStat interrupt, register 0x37[4]
	 */
	/* 60 * 50 = 3 secs */
	for (loops = 60 ; loops ; loops--) {
		if (chgr->prop_mode_en) {
			dev_info(&chgr->client->dev,
				 "PROP_MODE: Proprietary Mode Enabled\n");
			break;
		}
		msleep(50);
	}
	if (!chgr->prop_mode_en)
		goto err_exit;

request_pwr:
	/*
	 * Step 5: Read TX potential power register (0xC4)
	 * [TX max power capability] in 0.5W units
	 */
	ret = chgr->reg_read_8(chgr, P9412_PROP_TX_POTEN_PWR_REG, &txpwr);
	txpwr = txpwr / 2;
	if ((ret == 0) && (txpwr >= HPP_MODE_PWR_REQUIRE)) {
		dev_info(&chgr->client->dev,
			 "PROP_MODE: Tx potential power=%dW\n", txpwr);
	} else {
		chgr->prop_mode_en = false;
		goto err_exit;
	}

	/*
	 * Step 6: Request xx W Neg power by writing 0xC5,
	 * then write 0x02 to 0x4F
	 */
	ret = chgr->reg_write_8(chgr, P9412_PROP_REQ_PWR_REG, req_pwr * 2);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to write pwr req register\n");
		chgr->prop_mode_en = false;
		goto err_exit;
	} else {
		dev_info(&chgr->client->dev, "request power=%dW\n", req_pwr);
	}
	/* Request power from TX based on PropReqPwr (0xC5) */
	ret = chgr->chip_set_cmd(chgr, PROP_REQ_PWR_CMD);
	if (ret) {
		dev_err(&chgr->client->dev,
			"PROP_MODE: fail to send PROP_REQ_PWR_CMD\n");
		chgr->prop_mode_en = false;
		goto err_exit;
	}

	for (i = 0; i < 30; i += 1) {
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
		if (!chgr->online)
			chgr->prop_mode_en = false;
	}

err_exit:
        /* check status */
	ret = chgr->chip_get_sys_mode(chgr, &val8);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_CURR_PWR_REG, &prop_cur_pwr);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_PWR_STEP_REG, &pwr_stp);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_STATUS_REG, &mode_sts);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_MODE_ERR_STS_REG, &err_sts);
	ret |= chgr->reg_read_8(chgr, P9412_CDMODE_STS_REG, &cdmode);
	ret |= chgr->reg_read_8(chgr, P9412_PROP_REQ_PWR_REG, &prop_req_pwr);

	pr_debug("%s PROP_MODE: en=%d,sys_mode=%02x,mode_sts=%02x,err_sts=%02x,"
		 "cdmode=%02x,pwr_stp=%02x,req_pwr=%02x,prop_cur_pwr=%02x,txpwr=%dW",
		 __func__, chgr->prop_mode_en, val8, mode_sts, err_sts,
		 cdmode, pwr_stp, prop_req_pwr, prop_cur_pwr, txpwr);

	if (!ret) {
		dev_info(&chgr->client->dev,
			 "PROP_MODE: en=%d,sys_mode=%02x,mode_sts=%02x,"
			 "err_sts=%02x,cdmode=%02x,pwr_stp=%02x,"
			 "req_pwr=%02x,prop_cur_pwr=%02x",
			 chgr->prop_mode_en, val8, mode_sts, err_sts,
			 cdmode, pwr_stp, prop_req_pwr, prop_cur_pwr);
	}


	if (!chgr->prop_mode_en) {
		int rc;

		rc = gvotable_cast_int_vote(chgr->dc_icl_votable, P9221_HPP_VOTER,
					    P9XXX_CDMODE_ENABLE_ICL_UA, false);
		if (rc <0)
			dev_err(&chgr->client->dev, "%s: cannot remove HPP voter (%d)\n",
				__func__, ret);
	}

	return chgr->prop_mode_en;
}

void p9221_chip_init_interrupt_bits(struct p9221_charger_data *chgr, u16 chip_id)
{
	chgr->ints.mode_changed_bit = P9221R5_STAT_MODECHANGED;
	chgr->ints.vrecton_bit = P9221R5_STAT_VRECTON;
	chgr->ints.vout_changed_bit = P9221R5_STAT_VOUTCHANGED;

	switch (chip_id) {
	case P9412_CHIP_ID:
		chgr->ints.over_curr_bit = P9412_STAT_OVC;
		chgr->ints.over_volt_bit = 0;      /* TODO: b/181191668*/
		chgr->ints.over_temp_bit = P9412_STAT_OVT;
		chgr->ints.over_uv_bit = 0;
		chgr->ints.cc_send_busy_bit = P9412_STAT_CCSENDBUSY;
		chgr->ints.cc_data_rcvd_bit = P9412_STAT_CCDATARCVD;
		chgr->ints.pp_rcvd_bit = P9412_STAT_PPRCVD;
		chgr->ints.cc_error_bit = P9412_STAT_CCERROR;
		chgr->ints.cc_reset_bit = 0;
		chgr->ints.propmode_stat_bit = P9412_PROP_MODE_STAT_INT;
		chgr->ints.cdmode_change_bit = P9412_CDMODE_CHANGE_INT;
		chgr->ints.cdmode_err_bit = P9412_CDMODE_ERROR_INT;
		chgr->ints.extended_mode_bit = 0;

		chgr->ints.hard_ocp_bit = P9412_STAT_OVC;
		chgr->ints.tx_conflict_bit = P9412_STAT_TXCONFLICT;
		chgr->ints.csp_bit = P9412_STAT_CSP;
		chgr->ints.rx_connected_bit = P9412_STAT_RXCONNECTED;
		chgr->ints.tx_fod_bit = P9412_STAT_TXFOD;
		chgr->ints.tx_underpower_bit = 0;
		chgr->ints.tx_uvlo_bit = 0;
		chgr->ints.pppsent_bit = P9412_STAT_PPPSENT;
		break;
	case P9382A_CHIP_ID:
		chgr->ints.over_curr_bit = P9221R5_STAT_OVC;
		chgr->ints.over_volt_bit = P9221R5_STAT_OVV;
		chgr->ints.over_temp_bit = P9221R5_STAT_OVT;
		chgr->ints.over_uv_bit = P9221R5_STAT_UV;
		chgr->ints.cc_send_busy_bit = P9221R5_STAT_CCSENDBUSY;
		chgr->ints.cc_data_rcvd_bit = P9221R5_STAT_CCDATARCVD;
		chgr->ints.pp_rcvd_bit = P9221R5_STAT_PPRCVD;
		chgr->ints.cc_error_bit = P9221R5_STAT_CCERROR;
		chgr->ints.cc_reset_bit = P9221R5_STAT_CCRESET;
		chgr->ints.propmode_stat_bit = 0;
		chgr->ints.cdmode_change_bit = 0;
		chgr->ints.cdmode_err_bit = 0;
		chgr->ints.extended_mode_bit = 0;

		chgr->ints.hard_ocp_bit = P9382_STAT_HARD_OCP;
		chgr->ints.tx_conflict_bit = P9382_STAT_TXCONFLICT;
		chgr->ints.csp_bit = P9382_STAT_CSP;
		chgr->ints.rx_connected_bit = P9382_STAT_RXCONNECTED;
		chgr->ints.tx_fod_bit = P9382_STAT_TXFOD;
		chgr->ints.tx_underpower_bit = P9382_STAT_TXUNDERPOWER;
		chgr->ints.tx_uvlo_bit = P9382_STAT_TXUVLO;
		chgr->ints.pppsent_bit = P9221R5_STAT_CCSENDBUSY;
		break;
	case P9222_CHIP_ID:
		chgr->ints.over_curr_bit = P9222_STAT_OVC;
		chgr->ints.over_volt_bit = P9222_STAT_OVV;
		chgr->ints.over_temp_bit = P9222_STAT_OVT;
		chgr->ints.over_uv_bit = 0;
		chgr->ints.cc_send_busy_bit = P9221R5_STAT_CCSENDBUSY;
		chgr->ints.cc_data_rcvd_bit = P9221R5_STAT_CCDATARCVD;
		chgr->ints.pp_rcvd_bit = P9222_STAT_PPRCVD;
		chgr->ints.cc_error_bit = P9222_STAT_CCERROR;
		chgr->ints.cc_reset_bit = 0;
		chgr->ints.propmode_stat_bit = 0;
		chgr->ints.cdmode_change_bit = 0;
		chgr->ints.cdmode_err_bit = 0;
		chgr->ints.extended_mode_bit = P9222_EXTENDED_MODE;

		chgr->ints.hard_ocp_bit = 0;
		chgr->ints.tx_conflict_bit = 0;
		chgr->ints.csp_bit = 0;
		chgr->ints.rx_connected_bit = 0;
		chgr->ints.tx_fod_bit = 0;
		chgr->ints.tx_underpower_bit = 0;
		chgr->ints.tx_uvlo_bit = 0;
		chgr->ints.pppsent_bit = 0;
		break;
	default:
		chgr->ints.over_curr_bit = P9221R5_STAT_OVC;
		chgr->ints.over_volt_bit = P9221R5_STAT_OVV;
		chgr->ints.over_temp_bit = P9221R5_STAT_OVT;
		chgr->ints.over_uv_bit = P9221R5_STAT_UV;
		chgr->ints.cc_send_busy_bit = P9221R5_STAT_CCSENDBUSY;
		chgr->ints.cc_data_rcvd_bit = P9221R5_STAT_CCDATARCVD;
		chgr->ints.pp_rcvd_bit = P9221R5_STAT_PPRCVD;
		chgr->ints.cc_error_bit = P9221R5_STAT_CCERROR;
		chgr->ints.cc_reset_bit = P9221R5_STAT_CCRESET;
		chgr->ints.propmode_stat_bit = 0;
		chgr->ints.cdmode_change_bit = 0;
		chgr->ints.cdmode_err_bit = 0;
		chgr->ints.extended_mode_bit = 0;

		chgr->ints.hard_ocp_bit = 0;
		chgr->ints.tx_conflict_bit = 0;
		chgr->ints.csp_bit = 0;
		chgr->ints.rx_connected_bit = 0;
		chgr->ints.tx_fod_bit = 0;
		chgr->ints.tx_underpower_bit = 0;
		chgr->ints.tx_uvlo_bit = 0;
		chgr->ints.pppsent_bit = 0;
		break;
	}


	chgr->ints.stat_limit_mask = (chgr->ints.over_uv_bit |
				      chgr->ints.over_temp_bit |
				      chgr->ints.over_volt_bit |
				      chgr->ints.over_curr_bit);
	chgr->ints.stat_cc_mask = (chgr->ints.cc_send_busy_bit |
				   chgr->ints.cc_reset_bit |
				   chgr->ints.pp_rcvd_bit |
				   chgr->ints.cc_error_bit |
				   chgr->ints.cc_data_rcvd_bit);
	chgr->ints.prop_mode_mask = (chgr->ints.propmode_stat_bit |
				     chgr->ints.cdmode_change_bit |
				     chgr->ints.cdmode_err_bit);
	chgr->ints.stat_rtx_mask = (chgr->ints.stat_limit_mask |
				    chgr->ints.stat_cc_mask |
				    chgr->ints.mode_changed_bit |
				    chgr->ints.vout_changed_bit |
				    chgr->ints.hard_ocp_bit |
				    chgr->ints.tx_conflict_bit |
				    chgr->ints.csp_bit |
				    chgr->ints.rx_connected_bit |
				    chgr->ints.tx_fod_bit |
				    chgr->ints.tx_underpower_bit |
				    chgr->ints.tx_uvlo_bit |
				    chgr->ints.pppsent_bit);
}

void p9221_chip_init_params(struct p9221_charger_data *chgr, u16 chip_id)
{
	switch (chip_id) {
	case P9412_CHIP_ID:
		chgr->reg_tx_id_addr = P9412_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9382_EPP_TX_MFG_CODE_REG;
		chgr->reg_packet_type_addr = P9412_COM_PACKET_TYPE_ADDR;
		chgr->reg_set_pp_buf_addr = P9412_PP_SEND_BUF_START;
		chgr->reg_get_pp_buf_addr = P9412_PP_RECV_BUF_START;
		chgr->set_cmd_ccactivate_bit = P9412_COM_CCACTIVATE;
		chgr->reg_set_fod_addr = P9221R5_FOD_REG;
		chgr->reg_q_factor_addr = P9221R5_EPP_Q_FACTOR_REG;
		chgr->reg_csp_addr = P9221R5_CHARGE_STAT_REG;
		chgr->reg_light_load_addr = 0;
		chgr->reg_mot_addr = P9412_MOT_REG;
		break;
	case P9382A_CHIP_ID:
		chgr->reg_tx_id_addr = P9382_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9382_EPP_TX_MFG_CODE_REG;
		chgr->reg_packet_type_addr = P9382A_COM_PACKET_TYPE_ADDR;
		chgr->reg_set_pp_buf_addr = P9382A_DATA_SEND_BUF_START;
		chgr->reg_get_pp_buf_addr = P9382A_DATA_RECV_BUF_START;
		chgr->set_cmd_ccactivate_bit = P9221R5_COM_CCACTIVATE;
		chgr->reg_set_fod_addr = P9221R5_FOD_REG;
		chgr->reg_q_factor_addr = P9221R5_EPP_Q_FACTOR_REG;
		chgr->reg_csp_addr = P9221R5_CHARGE_STAT_REG;
		chgr->reg_light_load_addr = 0;
		chgr->reg_mot_addr = 0;
		break;
	case P9222_CHIP_ID:
		chgr->reg_tx_id_addr = P9222RE_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9222RE_TX_MFG_CODE_REG;
		chgr->reg_packet_type_addr = P9222RE_COM_PACKET_TYPE_ADDR;
		chgr->reg_set_pp_buf_addr = P9222RE_PP_SEND_BUF_START;
		chgr->reg_get_pp_buf_addr = P9222RE_PP_RECV_BUF_START;
		chgr->set_cmd_ccactivate_bit = P9222RE_COM_CCACTIVATE;
		chgr->reg_set_fod_addr = P9222RE_FOD_REG;
		chgr->reg_q_factor_addr = P9222RE_EPP_Q_FACTOR_REG;
		chgr->reg_csp_addr = P9222RE_CHARGE_STAT_REG;
		chgr->reg_light_load_addr = P9222_RX_CALIBRATION_LIGHT_LOAD;
		chgr->reg_mot_addr = 0;
		break;
	default:
		chgr->reg_tx_id_addr = P9221R5_PROP_TX_ID_REG;
		chgr->reg_tx_mfg_code_addr = P9221R5_EPP_TX_MFG_CODE_REG;
		chgr->reg_packet_type_addr = 0;
		chgr->reg_set_pp_buf_addr = P9221R5_DATA_SEND_BUF_START;
		chgr->reg_get_pp_buf_addr = P9221R5_DATA_RECV_BUF_START;
		chgr->set_cmd_ccactivate_bit = P9221R5_COM_CCACTIVATE;
		chgr->reg_set_fod_addr = P9221R5_FOD_REG;
		chgr->reg_q_factor_addr = P9221R5_EPP_Q_FACTOR_REG;
		chgr->reg_csp_addr = P9221R5_CHARGE_STAT_REG;
		chgr->reg_light_load_addr = 0;
		chgr->reg_mot_addr = 0;
		break;
	}
}

int p9221_chip_init_funcs(struct p9221_charger_data *chgr, u16 chip_id)
{
	chgr->chip_get_iout = p9xxx_chip_get_iout;
	chgr->chip_get_vout = p9xxx_chip_get_vout;
	chgr->chip_set_cmd = p9xxx_chip_set_cmd_reg;
	chgr->chip_get_op_freq = p9xxx_chip_get_op_freq;
	chgr->chip_get_vrect = p9xxx_chip_get_vrect;
	chgr->chip_get_vcpout = p9xxx_chip_get_vcpout;

	switch (chip_id) {
	case P9412_CHIP_ID:
		chgr->rtx_state = RTX_AVAILABLE;
		chgr->rx_buf_size = P9412_DATA_BUF_SIZE;
		chgr->tx_buf_size = P9412_DATA_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9412_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9412_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9412_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9412_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9412_chip_get_die_temp;
		chgr->chip_get_vout_max = p9412_chip_get_vout_max;
		chgr->chip_set_vout_max = p9412_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9412_chip_tx_mode;
		chgr->chip_get_data_buf = p9412_get_data_buf;
		chgr->chip_set_data_buf = p9412_set_data_buf;
		chgr->chip_get_cc_recv_size = p9412_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9412_set_cc_send_size;
		chgr->chip_get_align_x = p9412_get_align_x;
		chgr->chip_get_align_y = p9412_get_align_y;
		chgr->chip_send_ccreset = p9412_send_ccreset;
		chgr->chip_send_eop = p9412_send_eop;
		chgr->chip_get_sys_mode = p9412_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9412_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9412_prop_mode_enable;
		chgr->chip_check_neg_power = p9xxx_check_neg_power;
		chgr->chip_send_txid = p9xxx_send_txid;
		chgr->chip_send_csp_in_txmode = p9xxx_send_csp_in_txmode;
		chgr->chip_capdiv_en = p9412_capdiv_en;
		chgr->chip_get_vcpout = p9412_chip_get_vcpout;
		break;
	case P9382A_CHIP_ID:
		chgr->rtx_state = RTX_AVAILABLE;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9221_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9221_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9382_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9382_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9221_chip_get_die_temp;
		chgr->chip_get_vout_max = p9832_chip_get_vout_max;
		chgr->chip_set_vout_max = p9832_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9382_chip_tx_mode;
		chgr->chip_get_data_buf = p9382_get_data_buf;
		chgr->chip_set_data_buf = p9382_set_data_buf;
		chgr->chip_get_cc_recv_size = p9221_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9382_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9221_send_eop;
		chgr->chip_get_sys_mode = p9221_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9221_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		chgr->chip_check_neg_power = p9xxx_check_neg_power;
		chgr->chip_send_txid = p9xxx_send_txid;
		chgr->chip_send_csp_in_txmode = p9xxx_send_csp_in_txmode;
		chgr->chip_capdiv_en = p9221_capdiv_en;
		break;
	case P9222_CHIP_ID:
		chgr->chip_get_iout = p9222_chip_get_iout;
		chgr->chip_get_vout = p9222_chip_get_vout;
		chgr->chip_get_op_freq = p9222_chip_get_op_freq;
		chgr->chip_get_vrect = p9222_chip_get_vrect;
		chgr->chip_set_cmd = p9222_chip_set_cmd_reg;

		chgr->rtx_state = RTX_NOTSUPPORTED;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9222_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9222_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9221_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9221_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9222_chip_get_die_temp;
		chgr->chip_get_vout_max = p9222_chip_get_vout_max;
		chgr->chip_set_vout_max = p9222_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9221_chip_tx_mode;
		chgr->chip_get_data_buf = p9222_get_data_buf;
		chgr->chip_set_data_buf = p9222_set_data_buf;
		chgr->chip_get_cc_recv_size = p9222_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9222_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9222_send_eop;
		chgr->chip_get_sys_mode = p9222_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9222_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		chgr->chip_check_neg_power = p9222_check_neg_power;
		chgr->chip_send_txid = p9221_send_txid;
		chgr->chip_send_csp_in_txmode = p9221_send_csp_in_txmode;
		chgr->chip_capdiv_en = p9221_capdiv_en;
		break;
	default:
		chgr->rtx_state = RTX_NOTSUPPORTED;
		chgr->rx_buf_size = P9221R5_DATA_RECV_BUF_SIZE;
		chgr->tx_buf_size = P9221R5_DATA_SEND_BUF_SIZE;

		chgr->chip_get_rx_ilim = p9221_chip_get_rx_ilim;
		chgr->chip_set_rx_ilim = p9221_chip_set_rx_ilim;
		chgr->chip_get_tx_ilim = p9221_chip_get_tx_ilim;
		chgr->chip_set_tx_ilim = p9221_chip_set_tx_ilim;
		chgr->chip_get_die_temp = p9221_chip_get_die_temp;
		chgr->chip_get_vout_max = p9221_chip_get_vout_max;
		chgr->chip_set_vout_max = p9221_chip_set_vout_max;
		chgr->chip_tx_mode_en = p9221_chip_tx_mode;
		chgr->chip_set_data_buf = p9221_set_data_buf;
		chgr->chip_get_data_buf = p9221_get_data_buf;
		chgr->chip_get_cc_recv_size = p9221_get_cc_recv_size;
		chgr->chip_set_cc_send_size = p9221_set_cc_send_size;
		chgr->chip_get_align_x = p9221_get_align_x;
		chgr->chip_get_align_y = p9221_get_align_y;
		chgr->chip_send_ccreset = p9221_send_ccreset;
		chgr->chip_send_eop = p9221_send_eop;
		chgr->chip_get_sys_mode = p9221_chip_get_sys_mode;
		chgr->chip_renegotiate_pwr = p9221_chip_renegotiate_pwr;
		chgr->chip_prop_mode_en = p9221_prop_mode_enable;
		chgr->chip_check_neg_power = p9xxx_check_neg_power;
		chgr->chip_send_txid = p9221_send_txid;
		chgr->chip_send_csp_in_txmode = p9221_send_csp_in_txmode;
		chgr->chip_capdiv_en = p9221_capdiv_en;
		break;
	}

	chgr->rx_buf = devm_kzalloc(chgr->dev, chgr->rx_buf_size, GFP_KERNEL);
	if (chgr->rx_buf == NULL)
		return -ENOMEM;

	chgr->tx_buf = devm_kzalloc(chgr->dev, chgr->tx_buf_size, GFP_KERNEL);
	if (chgr->tx_buf == NULL)
		return -ENOMEM;

	return 0;
}

#if IS_ENABLED(CONFIG_GPIOLIB)
int p9xxx_gpio_set_value(struct p9221_charger_data *chgr, int gpio, int value)
{
	if (gpio <= 0)
		return -EINVAL;

	logbuffer_log(chgr->log, "%s: set gpio %d to %d\n", __func__, gpio, value);
	gpio_set_value_cansleep(gpio, value);

	return 0;
}

static int p9xxx_gpio_get_direction(struct gpio_chip *chip,
				    unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int p9xxx_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	//struct p9221_charger_data *charger = gpiochip_get_data(chip);
	int value = 0;

	switch (offset) {
	case P9XXX_GPIO_CPOUT_EN:
		break;
	case P9412_GPIO_CPOUT21_EN:
		// read cap divider
		break;
	case P9XXX_GPIO_CPOUT_CTL_EN:
		break;
	default:
		break;
	}

	return value;
}

#define P9412_BPP_VOUT_DFLT	5000
#define P9412_BPP_WLC_OTG_VOUT	5200
static void p9xxx_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct p9221_charger_data *charger = gpiochip_get_data(chip);
	int ret = 0;
	u8 mode;

	switch (offset) {
	case P9XXX_GPIO_CPOUT_EN:
		/* take offline (if online) and set/reset QI_EN_L */
		ret = gvotable_cast_bool_vote(charger->wlc_disable_votable,
					      CPOUT_EN_VOTER, !value);
		break;
	case P9412_GPIO_CPOUT21_EN:
		/* TODO: no-op for FW38+ */
		mode = !!value ? CDMODE_CAP_DIV_MODE : 0;
		ret = p9412_capdiv_en(charger, mode);
		break;
	case P9XXX_GPIO_CPOUT_CTL_EN:
		if (p9221_is_epp(charger))
			break;
		/* No need if DD is triggered */
		if (charger->trigger_power_mitigation)
			break;
		/* b/174068520: set vout to 5.2 for BPP_WLC_RX+OTG */
		if (value)
			ret = charger->chip_set_vout_max(charger, P9412_BPP_WLC_OTG_VOUT);
		else if (value == 0)
			ret = charger->chip_set_vout_max(charger, P9412_BPP_VOUT_DFLT);
		break;
	case P9XXX_GPIO_VBUS_EN:
		if (charger->pdata->wlc_en < 0)
			break;
		value = (!!value) ^ charger->pdata->wlc_en_act_low;
		gpio_direction_output(charger->pdata->wlc_en, value);
		break;
	case P9XXX_GPIO_DC_SW_EN:
		ret = p9xxx_gpio_set_value(charger, charger->pdata->dc_switch_gpio, value);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pr_debug("%s: GPIO offset=%d value=%d ret:%d\n", __func__,
		 offset, value, ret);

	if (ret < 0)
		dev_err(&charger->client->dev, "GPIO%d: value=%d ret:%d\n",
			offset, value, ret);
}

void p9xxx_gpio_init(struct p9221_charger_data *charger)
{
	charger->gpio.owner = THIS_MODULE;
	charger->gpio.label = "p9xxx_gpio";
	charger->gpio.get_direction = p9xxx_gpio_get_direction;
	charger->gpio.get = p9xxx_gpio_get;
	charger->gpio.set = p9xxx_gpio_set;
	charger->gpio.base = -1;
	charger->gpio.ngpio = P9XXX_NUM_GPIOS;
	charger->gpio.can_sleep = true;
}
#endif
