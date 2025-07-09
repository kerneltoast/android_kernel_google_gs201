/*
 * P9221 Wireless Charger Driver
 *
 * Copyright 2017-2022 Google LLC
 *
 */

#include <linux/device.h>
#include <linux/crc8.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/alarmtimer.h>
#include "p9221_charger.h"
#include "p9221-dt-bindings.h"
#include "google_dc_pps.h"
#include "google_bms.h"
#include "google_psy.h"
#include <linux/debugfs.h>

#define P9221R5_OVER_CHECK_NUM		3
#define MFG_CHK_COUNT_MAX		30
#define OVC_LIMIT			1
#define OVC_THRESHOLD			1400000
#define OVC_BACKOFF_LIMIT		900000
#define OVC_BACKOFF_AMOUNT		100000

#define WLC_ALIGNMENT_MAX		100
#define WLC_CURRENT_FILTER_LENGTH	10
#define WLC_ALIGN_DEFAULT_SCALAR	4
#define WLC_ALIGN_IRQ_THRESHOLD		10
#define WLC_ALIGN_DEFAULT_HYSTERESIS	5000
#define WLC_ALIGN_CURRENT_THRESHOLD	450
#define WLC_ALIGN_DEFAULT_SCALAR_LOW_CURRENT	    200
#define WLC_ALIGN_DEFAULT_SCALAR_HIGH_CURRENT	    118
#define WLC_ALIGN_DEFAULT_OFFSET_LOW_CURRENT	    125000
#define WLC_ALIGN_DEFAULT_OFFSET_HIGH_CURRENT	    139000
#define HPP_FOD_VOUT_THRESHOLD_UV	17500000
#define WLC_ALIGN_LOWER_LIMIT		(0)
#define WLC_ALIGN_UPPER_LIMIT		(90)

#define WLC_HPP_SOC_LIMIT	80
#define PROP_MODE_PWR_DEFAULT	30

#define REENABLE_RTX_DELAY	3000
#define P9XXX_CHK_RP_DELAY_MS	200

#define P9XXX_VOUT_5480MV	5480
#define P9XXX_VOUT_5000MV	5000
#define P9XXX_VOUT_10000MV	10000
#define P9XXX_FOD_CHK_DELAY_MS	2000

#define P9XXX_SET_RF_DELAY_MS	330

enum wlc_align_codes {
	WLC_ALIGN_CHECKING = 0,
	WLC_ALIGN_MOVE,
	WLC_ALIGN_CENTERED,
	WLC_ALIGN_ERROR,
};

enum wlc_chg_mode {
	WLC_BPP = 0,
	WLC_BPP_LV,
	WLC_EPP,
	WLC_GPP,
	WLC_EPP_COMP,
	WLC_EPP_IOP,
	WLC_HPP,
	WLC_HPP_HV,
};

#define BCL_WLC 	"BCL_WLC"
#define BCL_DEV_VOTER 	"BCL_DEV_VOTER"
#define BCL_WLC_VOTE	0

enum wlc_bcl_rtx_mode {
       WLC_ENABLED_TX,
       WLC_DISABLED_TX,
};


#define P9221_CRC8_POLYNOMIAL		0x07	/* (x^8) + x^2 + x + 1 */
DECLARE_CRC8_TABLE(p9221_crc8_table);

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger);
static void p9221_icl_ramp_start(struct p9221_charger_data *charger);
static void p9221_charge_stats_init(struct p9221_charge_stats *chg_data);
static void p9221_dump_charge_stats(struct p9221_charger_data *charger);
static bool p9221_check_feature(struct p9221_charger_data *charger, u64 ft);
static const char *p9221_get_tx_id_str(struct p9221_charger_data *charger);
static int p9221_set_bpp_vout(struct p9221_charger_data *charger);
static int p9221_set_hpp_dc_icl(struct p9221_charger_data *charger, bool enable);
static int p9221_ll_check_id(struct p9221_charger_data *charger);
static int p9221_has_dc_in(struct p9221_charger_data *charger);
static bool p9xxx_find_votable(struct p9221_charger_data *charger);
static void p9221_init_align(struct p9221_charger_data *charger);
static bool p9xxx_rtx_gpio_is_state(struct p9221_charger_data *charger, enum p9xxx_rtx_gpio_state state);
static int check_hpp_fod_level(struct p9221_charger_data *charger, int dc_voltage);

static char *align_status_str[] = {
	"...", "M2C", "OK", "-1"
};

static size_t p9221_hex_str(u8 *data, size_t len, char *buf, size_t max_buf,
			    bool msbfirst)
{
	int i;
	int blen = 0;
	u8 val;

	for (i = 0; i < len; i++) {
		if (msbfirst)
			val = data[len - 1 - i];
		else
			val = data[i];
		blen += scnprintf(buf + (i * 3), max_buf - (i * 3),
				  "%02x ", val);
	}
	return blen;
}

static int p9221_reg_read_n(struct p9221_charger_data *charger, u16 reg,
			    void *buf, size_t n)
{
	int ret;
	struct i2c_msg msg[2];
	u8 wbuf[2];

	msg[0].addr = charger->client->addr;
	msg[0].flags = charger->client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = wbuf;

	wbuf[0] = (reg & 0xFF00) >> 8;
	wbuf[1] = (reg & 0xFF);

	msg[1].addr = charger->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = n;
	msg[1].buf = buf;

	mutex_lock(&charger->io_lock);
	ret = i2c_transfer(charger->client->adapter, msg, 2);
	mutex_unlock(&charger->io_lock);

	if (ret < 0) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : ret;

		dev_err(&charger->client->dev,
			"i2c read error, reg:%x, ret:%d (%d)\n",
			reg, ret, nret);
		return nret;
	}

	if (charger->enable_i2c_debug) {
		const int buf_size = I2C_LOG_NUM * 3 + 1;
		int i, len = n;

		if (!charger->i2c_rxdebug_buf)
			charger->i2c_rxdebug_buf = kmalloc(buf_size, GFP_KERNEL);

		for (i = 0; len > 0; i++) {
			const int count = len > I2C_LOG_NUM ? I2C_LOG_NUM : len;
			const int offset = i * I2C_LOG_NUM;

			p9221_hex_str((u8 *)buf + offset, count,
				      charger->i2c_rxdebug_buf, buf_size, 0);
			dev_info(&charger->client->dev,
				 "i2c read %d bytes from reg %04x, offset: %04x: %s\n",
				 count, reg, offset, charger->i2c_rxdebug_buf);
			len -= I2C_LOG_NUM;
		}
	}

	return (ret == 2) ? 0 : -EIO;
}

static int p9221_reg_read_16(struct p9221_charger_data *charger, u16 reg,
			     u16 *val)
{
	u8 buf[2];
	int ret;

	ret = p9221_reg_read_n(charger, reg, buf, 2);
	if (ret == 0)
		*val = (buf[1] << 8) | buf[0];
	return ret;
}

static int p9221_reg_read_8(struct p9221_charger_data *charger,
			    u16 reg, u8 *val)
{
	return p9221_reg_read_n(charger, reg, val, 1);
}

static int p9221_reg_write_n(struct p9221_charger_data *charger, u16 reg,
			     const void *buf, size_t n)
{
	int ret;
	u8 *data;
	int datalen = 2 + n;

	data = kmalloc(datalen, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data[0] = reg >> 8;
	data[1] = reg & 0xFF;
	memcpy(&data[2], buf, n);

	mutex_lock(&charger->io_lock);
	ret = i2c_master_send(charger->client, data, datalen);
	mutex_unlock(&charger->io_lock);
	kfree(data);

	if (ret < datalen) {
		/*
		 * Treat -ENOTCONN as -ENODEV to suppress the get/set
		 * prop warnings.
		 */
		int nret = (ret == -ENOTCONN) ? -ENODEV : -EIO;

		dev_err(&charger->client->dev,
			"%s: i2c write error, reg: 0x%x, n: %zd ret: %d (%d)\n",
			__func__, reg, n, ret, nret);
		return nret;
	}

	if (charger->enable_i2c_debug) {
		const int buf_size = I2C_LOG_NUM * 3 + 1;
		int i, len = n;

		if (!charger->i2c_txdebug_buf)
			charger->i2c_txdebug_buf = kmalloc(buf_size, GFP_KERNEL);

		for (i = 0; len > 0; i++) {
			const int count = len > I2C_LOG_NUM ? I2C_LOG_NUM : len;
			const int offset = i * I2C_LOG_NUM;

			p9221_hex_str((u8 *)buf + offset, count,
				      charger->i2c_txdebug_buf, buf_size, 0);
			dev_info(&charger->client->dev,
				 "i2c write %d bytes to reg %04x, offset: %04x: %s\n",
				 count, reg, offset, charger->i2c_txdebug_buf);
			len -= I2C_LOG_NUM;
		}
	}

	return 0;
}

static int p9221_reg_write_16(struct p9221_charger_data *charger, u16 reg,
			      u16 val)
{
	return p9221_reg_write_n(charger, reg, &val, 2);
}

static int p9221_reg_write_8(struct p9221_charger_data *charger, u16 reg,
			     u8 val)
{
	return p9221_reg_write_n(charger, reg, &val, 1);
}

static bool p9221_is_hpp(struct p9221_charger_data *charger)
{
	int ret;
	uint8_t reg = 0;

	ret = charger->chip_get_sys_mode(charger, &reg);

	return ((ret == 0) && (reg == P9XXX_SYS_OP_MODE_PROPRIETARY));
}

static bool p9221_check_wpc_rev13(struct p9221_charger_data *charger)
{
	int ret;
	u8 val8;

	ret = p9221_reg_read_8(charger, P9412_WPC_SPEC_REV_REG, &val8);
	if (ret < 0)
		return false;

	if (val8 == P9XXX_WPC_REV_13) {
		logbuffer_log(charger->log, "WPC rev is %#02x", val8);
		return true;
	}

	if (charger->is_mfg_google || charger->mfg == WLC_MFG_108_FOR_GOOGLE)
		return true;

	return false;
}

bool p9221_is_epp(struct p9221_charger_data *charger)
{
	int ret;
	u32 vout_mv;
	u32 vout_uv;
	uint8_t reg;

	if (!charger->online)
		return false;
	if (charger->fake_force_epp > 0)
		return true;
	if (charger->force_bpp)
		return false;

	/*
	 *  NOTE: mfg may be zero due to race condition during bringup. will
	 *  check once more if mfg == 0.
	 */
	if (charger->mfg == 0) {
		ret = p9xxx_chip_get_tx_mfg_code(charger, &charger->mfg);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", ret);
	}

	charger->is_mfg_google = charger->mfg == WLC_MFG_GOOGLE;

	ret = charger->chip_get_sys_mode(charger, &reg);
	if (ret == 0)
		return ((reg == P9XXX_SYS_OP_MODE_WPC_EXTD) ||
			(reg == P9XXX_SYS_OP_MODE_PROPRIETARY));

	dev_err(&charger->client->dev, "Could not read mode: %d\n",
		ret);

	/* Check based on power supply voltage */
	ret = charger->chip_get_vout(charger, &vout_mv);
	if (ret) {
		dev_err(&charger->client->dev, "Could read VOUT_ADC, %d\n",
			ret);
		goto out;
	}
	vout_uv = P9221_MA_TO_UA(vout_mv);

	dev_info(&charger->client->dev, "Voltage is %duV\n", vout_uv);
	if (vout_uv > P9221_EPP_THRESHOLD_UV)
		return true;

out:
	/* Default to BPP otherwise */
	return false;
}

bool p9xxx_is_capdiv_en(struct p9221_charger_data *charger)
{
	int ret;
	u8 cdmode, mode_reg;

	if (charger->chip_id < P9412_CHIP_ID)
		return false;

	/* 9530 doesn't require capdiv mode */
	if (charger->chip_id == RA9530_CHIP_ID) {
		ret = charger->chip_get_sys_mode(charger, &mode_reg);
		if (ret == 0 && mode_reg == P9XXX_SYS_OP_MODE_PROPRIETARY)
			return true;
		return false;
	}

	ret = charger->reg_read_8(charger, P9412_CDMODE_STS_REG, &cdmode);
	if ((ret == 0) && (cdmode & CDMODE_CAP_DIV_MODE))
		return true;

	return false;
}
static int p9xxx_check_iop_fod_by_mfg(struct p9221_charger_data *charger)
{
	int i;

	for (i = 0; i < charger->pdata->fod_iop_mfg_num; i++) {
		if (charger->mfg == charger->pdata->fod_iop_mfg[i]) {
			dev_info(&charger->client->dev, "mfg is 0x%04x == 0x%04x\n",
				charger->mfg, charger->pdata->fod_iop_mfg[i]);
			return WLC_EPP_IOP;
		}
	}

	return WLC_EPP;
}

static int p9221_check_fod_by_fsw(struct p9221_charger_data *charger)
{
	const int freq_high_thres = charger->pdata->fod_fsw_high;
	const int freq_low_thres = charger->pdata->fod_fsw_low;
	u32 vout_mv = 0, freq_khz = 0;
	int ret;

	ret = charger->chip_get_vout_max(charger, &vout_mv);
	if (ret == 0 && vout_mv == P9XXX_VOUT_5000MV)
		return WLC_BPP;

	if (!charger->is_mfg_google || freq_high_thres < 0 || freq_low_thres < 0)
		return WLC_EPP;

	ret = charger->chip_get_op_freq(charger, &freq_khz);
	if (ret != 0)
		return WLC_EPP;
	if (freq_khz < freq_low_thres ||
	    (charger->fod_mode == WLC_EPP_COMP && freq_khz < freq_high_thres))
		return WLC_EPP_COMP;

	return WLC_EPP;
}

static void p9221_write_fod(struct p9221_charger_data *charger)
{
	int mode = WLC_BPP;
	u8 *fod = NULL;
	int fod_count = charger->pdata->fod_num;
	int ret;
	int retries = 3;
	int vout_mv;
	u32 vout;
	static char *wlc_mode[] = { "BPP", "BPP_LV", "EPP", "GPP", "EPP_COMP", "EPP_IOP",
				    "HPP_0", "HPP_1", "HPP_2", "HPP_3",
				    "HPP_4", "HPP_5", "HPP_6", "HPP_7" };

	mutex_lock(&charger->fod_lock);

	if (charger->fod_cnt)
		goto done;

	if (charger->no_fod)
		goto no_fod;

	if (!charger->pdata->fod_num &&
            !charger->pdata->fod_epp_num &&
            !charger->pdata->fod_hpp_num)
		goto no_fod;

	/* Default to BPP FOD */
	if (charger->pdata->fod_num)
		fod = charger->pdata->fod;

	ret = charger->chip_get_vout_max(charger, &vout_mv);
	if (p9221_is_epp(charger) && charger->pdata->fod_epp_num && vout_mv > 5500) {
		mode = WLC_EPP;
		if (charger->pdata->fod_fsw)
			mode = p9221_check_fod_by_fsw(charger);
		if (charger->pdata->fod_iop_mfg_num > 0)
			mode = p9xxx_check_iop_fod_by_mfg(charger);
		if (charger->pdata->fod_gpp_num > 0 && charger->mfg == WLC_MFG_GOOGLE)
			mode = WLC_GPP;
		if (mode == WLC_EPP) {
			fod = charger->pdata->fod_epp;
			fod_count = charger->pdata->fod_epp_num;
		} else if (mode == WLC_EPP_COMP) {
			fod = charger->pdata->fod_epp_comp;
			fod_count = charger->pdata->fod_epp_comp_num;
		} else if (mode == WLC_EPP_IOP) {
			fod = charger->pdata->fod_epp_iop;
			fod_count = charger->pdata->fod_epp_iop_num;
		} else if (mode == WLC_GPP) {
			/*
			 * Prevent chip damage during jiggling test, set frequency limit
			 * (0: Disable frequency limit function)
			 */
			p9xxx_chip_set_freq_limit(charger, charger->pdata->lowest_fsw_khz);
			fod = charger->pdata->fod_gpp;
			fod_count = charger->pdata->fod_gpp_num;
		}
	}

	if (!p9221_is_epp(charger) && charger->pdata->fod_lv_num && vout_mv < 5500) {
		fod = charger->pdata->fod_lv;
		fod_count = charger->pdata->fod_lv_num;
		mode = WLC_BPP_LV;
	}

	ret = charger->chip_get_vout(charger, &vout);
	charger->hpp_fod_level = (ret == 0) ? check_hpp_fod_level(charger, vout * 1000) : 0;
	if (p9xxx_is_capdiv_en(charger) && charger->pdata->nb_hpp_fod_vol > 0 && vout > 12000) {
		fod = charger->pdata->hpp_fods[charger->hpp_fod_level].fod;
		fod_count = charger->pdata->hpp_fods[charger->hpp_fod_level].num;
		mode = WLC_HPP + charger->hpp_fod_level;
	}

	if (mode == charger->fod_mode)
		goto done;

	charger->fod_mode = mode;

	if (!fod)
		goto no_fod;

	while (retries) {
		char s[P9221R5_NUM_FOD * 3 + 1];
		u8 fod_read[P9221R5_NUM_FOD];

		dev_info(&charger->client->dev,
			 "Writing %s FOD (n=%d reg=%02x try=%d)\n",
			 wlc_mode[mode], fod_count,
			 charger->reg_set_fod_addr, retries);

		ret = p9xxx_chip_set_fod_reg(charger, fod, fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not write FOD: %d\n", ret);
			goto unlock;
		}

		/* Verify the FOD has been written properly */
		ret = p9xxx_chip_get_fod_reg(charger, fod_read, fod_count);
		if (ret) {
			dev_err(&charger->client->dev,
				"Could not read back FOD: %d\n", ret);
			goto unlock;
		}

		if (memcmp(fod, fod_read, fod_count) == 0)
			goto done;

		p9221_hex_str(fod_read, fod_count, s, sizeof(s), 0);
		dev_err(&charger->client->dev,
			"FOD verify error, read: %s\n", s);

		retries--;
		msleep(100);
	}

no_fod:
	dev_warn(&charger->client->dev, "FOD not set! bpp:%d epp:%d hpp:%d hpp_hv:%d r:%d\n",
		 charger->pdata->fod_num, charger->pdata->fod_epp_num,
		 charger->pdata->fod_hpp_num, charger->pdata->fod_hpp_hv_num, retries);
done:
	if (charger->pdata->fod_fsw)
		mod_delayed_work(system_wq, &charger->chk_fod_work,
				 msecs_to_jiffies(P9XXX_FOD_CHK_DELAY_MS));
unlock:
	mutex_unlock(&charger->fod_lock);
}

#define CC_DATA_LOCK_MS		250

static int p9221_send_data(struct p9221_charger_data *charger)
{
	int ret;
	ktime_t now = get_boot_msec();

	if (charger->cc_data_lock.cc_use &&
	    charger->cc_data_lock.cc_rcv_at != 0 &&
	    (now - charger->cc_data_lock.cc_rcv_at > CC_DATA_LOCK_MS))
		charger->cc_data_lock.cc_use = false;

	if (charger->cc_data_lock.cc_use)
		return -EBUSY;

	if (charger->tx_busy)
		return -EBUSY;

	charger->tx_busy = true;

	mutex_lock(&charger->cmd_lock);

	ret = charger->chip_set_data_buf(charger, charger->tx_buf, charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load tx %d\n", ret);
		goto error;
	}

	ret = charger->chip_set_cc_send_size(charger, charger->tx_len);
	if (ret) {
		dev_err(&charger->client->dev, "Failed to load txsz %d\n", ret);
		goto error;
	}

	ret = charger->chip_set_cmd(charger, charger->set_cmd_ccactivate_bit);
	if (ret)
		goto error;

	mutex_unlock(&charger->cmd_lock);
	return ret;

error:
	mutex_unlock(&charger->cmd_lock);
	charger->tx_busy = false;
	return ret;
}

static int p9221_send_ccreset(struct p9221_charger_data *charger);

/* call with lock on mutex_lock(&charger->stats_lock) */
static int p9221_send_csp(struct p9221_charger_data *charger, u8 stat)
{
	int ret = 0;
	const bool no_csp = charger->ben_state &&
			    charger->ints.pppsent_bit &&
			    charger->com_busy;

	if (no_csp) {
		charger->last_capacity = -1;
		if (charger->com_busy >= COM_BUSY_MAX) {
			if (p9221_send_ccreset(charger) == 0)
				charger->com_busy = 0;
		} else {
			charger->com_busy += 1;
		}
		logbuffer_log(charger->rtx_log,
			     "com_busy=%d, did not send csp",
			     charger->com_busy);
		return ret;
	}

	mutex_lock(&charger->cmd_lock);

	if (charger->ben_state) {
		ret = charger->chip_send_csp_in_txmode(charger, stat);
		if (ret == 0)
			charger->com_busy += 1;
	}

	if (charger->online) {
		int wcin;

		wcin = p9221_has_dc_in(charger);
		if (wcin <= 0 || !charger->chip_is_calibrated(charger)) {
			charger->last_capacity = -1;
			dev_dbg(&charger->client->dev,
				"skip to send CSP=%d wcin=%d\n", stat, wcin);
			goto exit;
		}
		ret = p9221_reg_write_8(charger, charger->reg_csp_addr, stat);
		if (ret == 0)
			ret = charger->chip_set_cmd(charger, P9221R5_COM_SENDCSP);
		if (ret < 0)
			dev_info(&charger->client->dev, "Send CSP status=%d (%d)\n",
				 stat, ret);
	}
exit:
	mutex_unlock(&charger->cmd_lock);
	return ret;
}

u8 p9221_crc8(u8 *pdata, size_t nbytes, u8 crc)
{
	return crc8(p9221_crc8_table, pdata, nbytes, crc);
}

static bool p9221_is_online(const struct p9221_charger_data *charger)
{
	return charger->online || charger->ben_state;
}

static int p9221_ready_to_read(struct p9221_charger_data *charger)
{
	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

	if (!p9221_is_online(charger))
		return -ENODEV;

	return 0;
}

static int set_renego_state(struct p9221_charger_data *charger, enum p9xxx_renego_state state)
{
	mutex_lock(&charger->renego_lock);
	if (state == P9XXX_AVAILABLE ||
	    charger->renego_state == P9XXX_AVAILABLE) {
		charger->renego_state = state;
		mutex_unlock(&charger->renego_lock);
		return 0;
	}
	mutex_unlock(&charger->renego_lock);

	dev_warn(&charger->client->dev, "Not allowed due to renego_state=%d\n", charger->renego_state);
	return -EAGAIN;
}

static int p9221_send_ccreset(struct p9221_charger_data *charger)
{
	set_renego_state(charger, P9XXX_AVAILABLE);
	return charger->chip_send_ccreset(charger);
}


static void p9221_abort_transfers(struct p9221_charger_data *charger)
{
	/* Abort all transfers */
	cancel_delayed_work(&charger->tx_work);
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = true;
	charger->rx_len = 0;
	set_renego_state(charger, P9XXX_AVAILABLE);
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
}

/*
 * Put the default ICL back to BPP, reset OCP voter, reset ALIGN voter
 * @pre charger && charger->dc_icl_votable && charger->client->dev
 * gvotable_cast_int_vote can handle NULL votable, so don't need check
 */
static void p9221_vote_defaults(struct p9221_charger_data *charger)
{
	int ret, ocp_icl;

	if (!charger->dc_icl_votable) {
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL - no votable\n");
		return;
	}

	ret = gvotable_cast_long_vote(charger->dc_icl_votable, P9221_WLC_VOTER,
				      P9221_DC_ICL_BPP_UA, true);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	ocp_icl = (charger->dc_icl_epp > 0) ?
		   charger->dc_icl_epp : P9221_DC_ICL_EPP_UA;

	ret = gvotable_cast_int_vote(charger->dc_icl_votable,
				     P9221_OCP_VOTER, ocp_icl, true);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not reset OCP DC_ICL voter %d\n", ret);

	/* TODO: convert all to gvotable_recast_ballot() */
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       P9382A_RTX_VOTER, 0, false);
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       DCIN_AICL_VOTER, 0, false);
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       HPP_DC_ICL_VOTER, 0, false);
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       DD_VOTER, 0, false);

	p9221_set_auth_dc_icl(charger, false);
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       P9221_RAMP_VOTER, 0, false);
	gvotable_cast_int_vote(charger->dc_icl_votable,
			       P9221_HPP_VOTER, 0, false);
	gvotable_cast_int_vote(charger->hda_tz_votable,
			       P9221_WLC_VOTER, 0, false);
	gvotable_cast_int_vote(charger->hda_tz_votable,
			       P9221_ALIGN_VOTER, 0, false);
	gvotable_cast_int_vote(charger->dc_avail_votable,
			       DD_VOTER, 0, false);
	gvotable_cast_int_vote(charger->hda_tz_votable,
			       P9221_WLC_VOTER, 0, false);
}

static int p9221_set_switch_reg(struct p9221_charger_data *charger, bool enable)
{
	u8 swreg;
	int ret = p9221_reg_read_8(charger, P9412_V5P0AP_SWITCH_REG, &swreg);
	if (ret < 0) {
		dev_warn(&charger->client->dev,
			 "Failed to read swreg (%d)\n", ret);
		return ret;
	}
	if (enable)
		swreg |= V5P0AP_SWITCH_EN;
	else
		swreg &= ~V5P0AP_SWITCH_EN;
	return p9221_reg_write_8(charger, P9412_V5P0AP_SWITCH_REG, swreg);
}

#define EPP_MODE_REQ_VOUT		12000
#define WLC_VOUT_RAMP_DOWN_MV		15300
#define WLC_VOUT_CFG_STEP		40		/* b/194346461 ramp down VOUT */
#define WLC_VOUT_MARGIN_MV		100
static int p9xxx_set_bypass_mode(struct p9221_charger_data *charger)
{
	const int req_pwr_mv = charger->de_epp_neg_pwr > 0 ?
			       charger->de_epp_neg_pwr : charger->pdata->epp_neg_pwr;
	const int req_pwr = req_pwr_mv * 2 / 1000;
	const int vout_target = WLC_VOUT_RAMP_DOWN_MV;
	int i, count, ret;
	u8 cdmode, currpwr;
	u32 vout_mv = 0, vout_now;

	if (!charger->online)
		return 0;
	if (charger->chip_id == P9412_CHIP_ID) {
		/* Check it's in Cap Div mode */
		ret = charger->reg_read_8(charger, P9412_CDMODE_STS_REG, &cdmode);
		if (ret || (cdmode & CDMODE_BYPASS_MODE))
			return ret;
		dev_info(&charger->client->dev, "cdmode_reg=%02x\n", cdmode);

		usleep_range(500 * USEC_PER_MSEC, 510 * USEC_PER_MSEC);
	}
	/* Ramp down WLC Vout to 15.3V */
	while (true) {
		ret = charger->chip_get_vout(charger, &vout_now);
		if (ret < 0 || vout_now == 0) {
			dev_err(&charger->client->dev, "%s: invalid vout %d\n", __func__, ret);
			return ret;
		}

		if (!vout_mv)
			vout_mv = vout_now;
		if (vout_mv < vout_target) {
			if (vout_now < vout_target + WLC_VOUT_MARGIN_MV) {
				dev_info(&charger->client->dev,
					 "%s: underflow vout=%d (target=%d)\n",
					 __func__, vout_now, vout_target);
				break;
			}
			dev_dbg(&charger->client->dev, "%s: vout_now=%d, (target=%d)\n",
				__func__, vout_now, vout_target);
			usleep_range(250 * USEC_PER_MSEC, 260 * USEC_PER_MSEC);
			continue;
		}

		vout_mv -= WLC_VOUT_CFG_STEP;

		ret = charger->chip_set_vout_max(charger, vout_mv);
		if (ret < 0) {
			dev_err(&charger->client->dev, "%s: cannot set vout %d\n", __func__, ret);
			return ret;
		} else {
			dev_info(&charger->client->dev, "%s: vout set to %d\n", __func__, vout_mv);
			usleep_range(250 * USEC_PER_MSEC, 260 * USEC_PER_MSEC);
		}
	}

	if (!charger->online)
		return 0;

	usleep_range(500 * USEC_PER_MSEC, 510 * USEC_PER_MSEC);
	for (count = 0; count < 3; count++) {
		/* Change the Requested Power to 15W */
		ret = charger->reg_write_8(charger, P9412_PROP_REQ_PWR_REG, req_pwr);
		if (ret == 0)
			ret = charger->chip_set_cmd(charger, PROP_REQ_PWR_CMD);
		if (ret)
			dev_warn(&charger->client->dev,
				 "Fail to request Tx power(%d)\n", ret);

		/* total 5 seconds wait and early exit when WLC offline */
		for (i = 0; i < 50; i += 1) {
			usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
			if (!charger->online) {
				dev_err(&charger->client->dev, "%s: WLC offline\n", __func__);
				return -ENODEV;
			}
		}

		/* Check PropCurrPwr and P9412 Vout */
		vout_mv = 0;
		currpwr = 0;
		ret = charger->chip_get_vout(charger, &vout_mv);
		ret |= charger->reg_read_8(charger, P9412_PROP_CURR_PWR_REG, &currpwr);
		dev_info(&charger->client->dev, "count=%d, currpwr=%02x, vout_mv=%u\n",
			 count, currpwr, vout_mv);
		if (ret == 0 && currpwr == req_pwr && vout_mv < EPP_MODE_REQ_VOUT)
			break;
	}

	if (count == 3) {
		dev_err(&charger->client->dev, "%s: timeout for exit from high power mode\n", __func__);
		return -ETIMEDOUT;
	}

	if (charger->chip_id == P9412_CHIP_ID) {
		/* Request Bypass mode */
		ret = charger->chip_capdiv_en(charger, CDMODE_BYPASS_MODE);
		if (ret) {
			u8 mode_sts = 0, err_sts = 0;
			int rc;

			rc = charger->reg_read_8(charger, P9412_PROP_MODE_STATUS_REG, &mode_sts);
			rc |= charger->reg_read_8(charger, P9412_PROP_MODE_ERR_STS_REG, &err_sts);
			dev_err(&charger->client->dev,
				"Fail to change to bypass mode(%d), rc=%d sts=%02x, err=%02x\n",
				ret, rc, mode_sts, err_sts);
		}
	}

	return ret;
}

static int p9221_reset_wlc_dc(struct p9221_charger_data *charger)
{
	const int dc_sw_gpio = charger->pdata->dc_switch_gpio;
	const int extben_gpio = charger->pdata->ext_ben_gpio;
	int ret;

	if (!charger->pdata->has_wlc_dc)
		return -EOPNOTSUPP;

	dev_dbg(&charger->client->dev, "%s start\n", __func__);
	charger->wlc_dc_enabled = false;

	usleep_range(500 * USEC_PER_MSEC, 510 * USEC_PER_MSEC);
	p9xxx_gpio_set_value(charger, dc_sw_gpio, 0);
	p9xxx_gpio_set_value(charger, extben_gpio, 0);

	p9221_set_switch_reg(charger, false);

	ret = p9221_set_hpp_dc_icl(charger, false);
	if (ret < 0)
		dev_warn(&charger->client->dev, "Cannot disable HPP_ICL (%d)\n", ret);

	gvotable_cast_int_vote(charger->dc_icl_votable, P9221_HPP_VOTER, 0, false);

	if (charger->chg_mode_votable) {
		gvotable_cast_long_vote(charger->chg_mode_votable,
					P9221_WLC_VOTER,
					0, false);
		charger->chg_mode_off = false;
	}

	ret = p9xxx_set_bypass_mode(charger);
	if (ret) {
		/*
		 * going to go offline and reset the state when
		 * 1. fail to change to bypass mode
		 * 2. WLC offline(normal)
		 * 3. MW wcin is 0 but WLC chip Vout > 0
		 */
		gvotable_cast_bool_vote(charger->wlc_disable_votable,
					P9221_HPP_VOTER, true);
		usleep_range(200 * USEC_PER_MSEC, 220 * USEC_PER_MSEC);
		gvotable_cast_bool_vote(charger->wlc_disable_votable,
					P9221_HPP_VOTER, false);
	} else {
		charger->prop_mode_en = false;
		charger->negotiation_complete = false;
		p9221_write_fod(charger);
	}
	if (charger->pdata->gpp_cmfet > 0) {
		charger->chip_set_cmfet(charger);
	} else {
		ret = p9xxx_chip_set_cmfet_reg(charger, charger->wlc_default_comcap);
		if (ret < 0 && ret != -ENOTSUPP)
			dev_warn(&charger->client->dev, "Fail to set comm cap(%d)\n", ret);
	}

	if (charger->alignment == -1)
		p9221_init_align(charger);

	charger->chip_set_ovp(charger, OVSET_EPP);

	return ret;
}

#define CHARGE_15W_VOUT_UV	12000000
#define CHARGE_15W_ILIM_UA	1270000

/* call holding mutex_lock(&charger->auth_lock); */
static int feature_set_dc_icl(struct p9221_charger_data *charger, u32 ilim_ua)
{
	const u32 delay = charger->pdata->icl_ramp_delay_ms == -1 ?
			  0 : charger->pdata->icl_ramp_delay_ms;
	charger->icl_ramp_alt_ua = ilim_ua > 0 ? ilim_ua : 0;
	p9221_set_auth_dc_icl(charger, false);
	p9221_icl_ramp_reset(charger);

	dev_info(&charger->client->dev, "ICL ramp set alarm %dms, %dua, ramp=%d\n",
		 delay, charger->icl_ramp_alt_ua, charger->icl_ramp);

	alarm_start_relative(&charger->icl_ramp_alarm, ms_to_ktime(delay));
	return 0;
}

/* call with mutex_lock(&charger->feat_lock); */
static int feature_15w_enable(struct p9221_charger_data *charger, bool enable)
{
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	int ret = 0;

	pr_debug("%s: enable=%d chip_id=%x\n", __func__, enable,
		 charger->pdata->chip_id);

	mutex_lock(&charger->auth_lock);

	/* wc_vol =12V & wc_cur = 1.27A */
	if (enable && !(chg_fts->session_features & WLCF_CHARGE_15W)) {
		const u32 vout_mv = P9221_UV_TO_MV(CHARGE_15W_VOUT_UV);

		if (charger->pdata->chip_id != P9412_CHIP_ID && !charger->pdata->enable_15w) {
			mutex_unlock(&charger->auth_lock);
			return -ENOTSUPP;
		}

		ret = charger->chip_set_vout_max(charger, vout_mv);
		if (ret == 0)
			ret = feature_set_dc_icl(charger, CHARGE_15W_ILIM_UA);
		if (ret == 0)
			ret = gvotable_cast_long_vote(charger->dc_icl_votable,
						      P9221_OCP_VOTER,
						      CHARGE_15W_ILIM_UA,
						      true);
	} else if (!enable && (chg_fts->session_features & WLCF_CHARGE_15W)) {
		const u32 vout_mv = P9221_UV_TO_MV(P9221_EPP_THRESHOLD_UV);
		const int ocp_icl = (charger->dc_icl_epp > 0) ?
				    charger->dc_icl_epp : P9221_DC_ICL_EPP_UA;
		int rc1, rc2, rc3;

		/* WLCF_CHARGE_15W is not not set on !P9412_CHIP_ID */

		/* offline might have reset this already */
		rc1 = gvotable_cast_long_vote(charger->dc_icl_votable,
					      P9221_OCP_VOTER, ocp_icl,
					      true);
		if (rc1 < 0)
			dev_err(&charger->client->dev, "15W: cannot reset ocp_icl (%d)", rc1);
		/* reset ramp */
		charger->icl_ramp_alt_ua = 0;
		rc2 = gvotable_cast_int_vote(charger->dc_icl_votable, DCIN_AICL_VOTER,
					     charger->icl_ramp_alt_ua, true);
		if (rc2 < 0)
			dev_err(&charger->client->dev, "15W: cannot reset ramp (%d)", rc2);
		/* reset VOUT will fail if online */
		rc3 = charger->chip_set_vout_max(charger, vout_mv);
		if (rc3 < 0)
			dev_dbg(&charger->client->dev, "15W: cannot reset vout (%d)", rc3);

		ret = rc1 < 0 || rc2 < 0 ? -EIO : 0;
	}
	mutex_unlock(&charger->auth_lock);
	return ret;
}

static int feature_gpp_15w_enable(struct p9221_charger_data *charger, bool enable)
{
	const int curr_icl = gvotable_get_int_vote(charger->dc_icl_votable, P9221_WLC_VOTER);
	const int icl = enable ? charger->pdata->gpp_icl : P9XXX_DC_ICL_EPP_1000;
	int ret = -EOPNOTSUPP;
	u8 val;

	if (curr_icl == icl)
		return 0;

	mutex_lock(&charger->auth_lock);

	if (!p9221_is_epp(charger))
		goto error_exit;
	if (charger->last_capacity > WLC_HPP_SOC_LIMIT && enable)
		goto error_exit;

	ret = charger->reg_read_8(charger, P9221R5_EPP_TX_GUARANTEED_POWER_REG, &val);
	if (ret) {
		dev_warn(&charger->client->dev, "Can't get guar_pwr\n");
		goto error_exit;
	} else if (val <= P9XXX_TX_GUAR_PWR_12W) {
		dev_dbg(&charger->client->dev, "15W: Tx guar_pwr=%dW too low\n", val / 2);
		ret = -EOPNOTSUPP;
		goto error_exit;
	}

	if (!charger->dc_icl_votable) {
		dev_warn(&charger->client->dev, "15W: Could not find DC_ICL votable\n");
		ret = -EIO;
		goto error_exit;
	}

	dev_dbg(&charger->client->dev, "%s: Voting ICL %duA\n", __func__, icl);
	ret = gvotable_cast_long_vote(charger->dc_icl_votable,
				      P9221_OCP_VOTER,
				      icl,
				      true);
	ret |= gvotable_cast_long_vote(charger->dc_icl_votable,
				      P9221_WLC_VOTER,
				      icl,
				      true);
	if (ret < 0)
		dev_err(&charger->client->dev, "15W: cannot set icl (%d)\n", ret);

error_exit:
	if (enable)
		p9221_set_auth_dc_icl(charger, false);
	mutex_unlock(&charger->auth_lock);

	return ret;
}

#define FEAT_SESSION_SUPPORTED \
	(WLCF_DREAM_DEFEND | WLCF_DREAM_ALIGN | WLCF_FAST_CHARGE | WLCF_CHARGE_15W)

/* handle the session properties here */
static int feature_update_session(struct p9221_charger_data *charger, u64 ft)
{
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	u64 session_features;
	int ret = 0;

	mutex_lock(&chg_fts->feat_lock);
	session_features = chg_fts->session_features;

	pr_debug("%s: ft=%llx", __func__, ft);

	/* change the valid (and add code if needed) */
	if (ft & ~FEAT_SESSION_SUPPORTED)
		dev_warn(charger->dev, "unsupported features ft=%llx\n", ft);

	if (ft & WLCF_DREAM_DEFEND)
		chg_fts->session_features |= WLCF_DREAM_DEFEND;
	else
		chg_fts->session_features &= ~WLCF_DREAM_DEFEND;

	if (ft & WLCF_DREAM_ALIGN) {
		chg_fts->session_features |= WLCF_DREAM_ALIGN;
	} else if (chg_fts->session_features & WLCF_DREAM_ALIGN) {
		/* TODO: kill aligmnent aid? */
		chg_fts->session_features &= ~WLCF_DREAM_ALIGN;
	}

	if (charger->pdata->gpp_enhanced)
		ret = feature_gpp_15w_enable(charger, ft & WLCF_FAST_CHARGE);
	if (ret < 0) {
		const bool enable = (ft & WLCF_FAST_CHARGE) != 0;

		dev_warn(&charger->client->dev, "error on feat GPP 15W ena=%d ret=%d\n",
			 enable, ret);
	} else if (ft & WLCF_FAST_CHARGE) {
		chg_fts->session_features |= WLCF_FAST_CHARGE;
	} else if (chg_fts->session_features & WLCF_FAST_CHARGE) {
		if (charger->online)
			dev_warn(&charger->client->dev,
				 "Cannot disable FAST_CHARGE while online\n");

		/*
		 * WLCF_FAST_CHARGE enables HPP when supported.
		 * TODO: gracefully degrade to 15W otherwise.
		 */
		chg_fts->session_features &= ~WLCF_FAST_CHARGE;
	}

	/* this might fail in interesting ways */
	ret = feature_15w_enable(charger, ft & WLCF_CHARGE_15W);
	if (ret < 0) {
		const bool enable = (ft & WLCF_CHARGE_15W) != 0;

		dev_warn(&charger->client->dev, "error on feat 15W ena=%d ret=%d\n",
			 enable, ret);

		/*
		 * -EINVAL, -ENOTSUPP or an I/O error.
		 * TODO report the failure in the session_features
		 */
	} else if (ft & WLCF_CHARGE_15W) {
		chg_fts->session_features |= WLCF_CHARGE_15W;
	} else {
		chg_fts->session_features &= ~WLCF_CHARGE_15W;
	}

	/* warn when a feature doesn't have a rule and align session_features */
	if (session_features != ft)
		logbuffer_log(charger->log, "session features %llx->%llx [%llx]",
			      session_features, ft, chg_fts->session_features);
	chg_fts->session_features = ft;

	mutex_unlock(&chg_fts->feat_lock);
	return 0;
}

static int p9221_capacity_raw(struct p9221_charger_data *charger)
{
	int val, ret;

	val = GPSY_GET_INT_PROP(charger->batt_psy, GBMS_PROP_CAPACITY_RAW, &ret);
	if (ret == 0)
		ret = qnum_toint(qnum_from_q8_8(val));

	return ret;
}

static void p9221_wcin_inlim(struct p9221_charger_data *charger)
{
	int msc_last;

	if (charger->pdata->wcin_inlim_en_gpio < 0)
		return;

	if (!charger->msc_last_votable)
		charger->msc_last_votable = gvotable_election_get_handle("MSC_LAST");

	msc_last = gvotable_get_current_int_vote(charger->msc_last_votable);

	if (charger->online && (msc_last == 1 || charger->last_capacity > 90) && !p9221_is_epp(charger))
		gpio_set_value_cansleep(charger->pdata->wcin_inlim_en_gpio, true);
	else
		gpio_set_value_cansleep(charger->pdata->wcin_inlim_en_gpio, false);

	dev_dbg(charger->dev, "wcin inlim check: online=%d, msc_last=%d, epp=%d\n",
		 charger->online, msc_last, p9221_is_epp(charger));
}

static void p9221_set_offline(struct p9221_charger_data *charger)
{
	if (charger->fod_cnt == 0) {
		/* already change to BPP or not doing power_mitigation */
		cancel_delayed_work(&charger->power_mitigation_work);
		charger->trigger_power_mitigation = false;
		charger->wait_for_online = false;
	} else {
		charger->wait_for_online = true;
		schedule_delayed_work(&charger->power_mitigation_work,
				      msecs_to_jiffies(
					  P9221_POWER_MITIGATE_DELAY_MS));
	}

	dev_info(&charger->client->dev, "Set offline\n");

	mutex_lock(&charger->stats_lock);
	charger->online = false;
	charger->online_at = 0;
	charger->check_rp = RP_NOTSET;
	cancel_delayed_work(&charger->charge_stats_hda_work);
	p9221_dump_charge_stats(charger);
	mutex_unlock(&charger->stats_lock);

	mutex_lock(&charger->auth_lock);
	charger->set_auth_icl = false;
	mutex_unlock(&charger->auth_lock);

	charger->sw_ramp_done = false;
	charger->force_bpp = false;
	charger->chg_on_rtx = false;
	charger->chg_on_cc_rtx = false;
	if (!charger->wait_for_online)
		p9221_reset_wlc_dc(charger);
	charger->prop_mode_en = false;
	charger->negotiation_complete = false;
	charger->hpp_hv = false;
	charger->fod_mode = -1;
	charger->extended_int_recv = false;
	set_renego_state(charger, P9XXX_AVAILABLE);

	/* Reset PP buf so we can get a new serial number next time around */
	charger->pp_buf_valid = false;
	memset(charger->pp_buf, 0, sizeof(charger->pp_buf));
	charger->rtx_csp = 0;

	p9221_abort_transfers(charger);
	cancel_delayed_work(&charger->dcin_work);

	/* Reset alignment value when charger goes offline */
	cancel_delayed_work(&charger->align_work);
	charger->align = WLC_ALIGN_ERROR;
	charger->align_count = 0;
	charger->alignment = -1;
	charger->alignment_capable = ALIGN_MFG_FAILED;
	charger->mfg = 0;
	charger->tx_id = 0;
	schedule_work(&charger->uevent_work);

	p9221_icl_ramp_reset(charger);
	del_timer(&charger->vrect_timer);

	/* clear all session features */
	if (!charger->wait_for_online) {
		feature_update_session(charger, WLCF_DISABLE_ALL_FEATURE);
	}

	p9221_vote_defaults(charger);
	if (charger->enabled)
		mod_delayed_work(system_wq, &charger->dcin_pon_work,
				 msecs_to_jiffies(P9221_DCIN_PON_DELAY_MS));

	logbuffer_log(charger->log, "offline\n");

	p9221_wcin_inlim(charger);

}

static void p9221_tx_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, tx_work.work);

	dev_info(&charger->client->dev, "timeout waiting for tx complete\n");

	set_renego_state(charger, P9XXX_AVAILABLE);
	charger->tx_busy = false;
	charger->tx_done = true;
	sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
	sysfs_notify(&charger->dev->kobj, NULL, "txdone");
}

static void p9221_vrect_timer_handler(struct timer_list *t)
{
	struct p9221_charger_data *charger = from_timer(charger,
							t, vrect_timer);

	if (charger->align == WLC_ALIGN_CHECKING) {
		charger->align = WLC_ALIGN_MOVE;
		logbuffer_log(charger->log, "align: state: %s",
			      align_status_str[charger->align]);
		schedule_work(&charger->uevent_work);
	}
	dev_info(&charger->client->dev,
		 "timeout waiting for VRECT, online=%d\n", charger->online);
	logbuffer_log(charger->log,
		"vrect: timeout online=%d", charger->online);

	mod_timer(&charger->align_timer,
		  jiffies + msecs_to_jiffies(P9221_ALIGN_TIMEOUT_MS));

	pm_relax(charger->dev);
}

static void p9221_align_timer_handler(struct timer_list *t)
{
	struct p9221_charger_data *charger = from_timer(charger,
							t, align_timer);

	charger->align = WLC_ALIGN_ERROR;
	logbuffer_log(charger->log, "align: timeout no IRQ");
	schedule_work(&charger->uevent_work);
}

#ifdef CONFIG_DC_RESET
/*
 * Offline disables ->qien_gpio: this worker re-enable it P9221_DCIN_TIMEOUT_MS
 * ms later to make sure that the WLC IC goes through a full reset.
 */
static void p9221_dcin_pon_work(struct work_struct *work)
{
	int ret;
	union power_supply_propval prop;
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, dcin_pon_work.work);

	if (!charger->dc_psy)
		return;

	ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_DC_RESET, &prop);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"unable to get DC_RESET, ret=%d\n", ret);
		return;
	}

	if (prop.intval != 0) {
		/* Signal DC_RESET when vout keeps on 1. */
		ret = power_supply_set_property(charger->dc_psy,
						POWER_SUPPLY_PROP_DC_RESET,
						&prop);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"unable to set DC_RESET, ret=%d", ret);

		schedule_delayed_work(&charger->dcin_pon_work,
			msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	}
}
#else
static void p9221_dcin_pon_work(struct work_struct *work)
{

}
#endif

static void p9221_dcin_work(struct work_struct *work)
{
	int res;
	u16 status_reg = 0;
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, dcin_work.work);

	res = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);

	if (res == 0 && charger->dcin_waitcnt) {
		charger->dcin_waitcnt--;
		schedule_delayed_work(&charger->dcin_work,
				      msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
		logbuffer_log(charger->log, "dc_in: check online=%d status=%04x",
			      charger->online, status_reg);
		return;
	}

	if (res == 0)
		logbuffer_log(charger->log, "dc_in: timeout online=%d status=%04x",
			      charger->online, status_reg);
	else
		logbuffer_log(charger->log, "dc_in: timeout online=%d res=%d",
			      charger->online, res);

	dev_info(&charger->client->dev, "timeout waiting for dc-in, online=%d\n",
		 charger->online);

	if (charger->online)
		p9221_set_offline(charger);

	power_supply_changed(charger->wc_psy);
	pm_relax(charger->dev);
}

static void force_set_fod(struct p9221_charger_data *charger)
{
	u8 fod[P9221R5_NUM_FOD] = { 0 };
	int ret;

	mutex_lock(&charger->fod_lock);

	if (p9221_is_hpp(charger)) {
		dev_info(&charger->client->dev,
			"power_mitigate: send EOP for revert to BPP\n");
		ret = charger->chip_send_eop(charger, P9221_EOP_REVERT_TO_BPP);
	} else {
		dev_info(&charger->client->dev, "power_mitigate: write 0 to fod\n");
		ret = p9xxx_chip_set_fod_reg(charger, fod, P9221R5_NUM_FOD);
	}
	if (ret)
		dev_err(&charger->client->dev,
			"power_mitigate: failed, ret=%d\n", ret);

	mutex_unlock(&charger->fod_lock);
}

static void p9221_power_mitigation_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, power_mitigation_work.work);
	int ret = 0;

	charger->wait_for_online = false;

	if (!p9221_is_online(charger)) {
		dev_info(&charger->client->dev, "power_mitigate: offline\n");
		charger->fod_cnt = 0;
		charger->trigger_power_mitigation = false;
		feature_update_session(charger, WLCF_DISABLE_ALL_FEATURE);
		power_supply_changed(charger->wc_psy);
		return;
	}

	if (!p9221_is_epp(charger)) {
		/* only for LL */
		if (charger->trigger_power_mitigation) {
			if (!charger->pdata->ll_vout_not_set) {
				dev_info(&charger->client->dev,
				      "power_mitigate: change Vout to %d mV and disable CMFET\n",
				      P9XXX_VOUT_5480MV);
				ret = charger->chip_set_vout_max(charger, P9XXX_VOUT_5480MV);
				if (ret < 0)
					dev_err(&charger->client->dev,
						"Fail to configure Vout to %d mV\n",
						P9XXX_VOUT_5480MV);
			}
			ret = p9xxx_chip_set_cmfet_reg(charger, charger->wlc_dd_comcap);
			if (ret < 0 && ret != -ENOTSUPP)
				dev_err(&charger->client->dev, "Fail to configure LL\n");
		}
		charger->fod_cnt = 0;
		dev_info(&charger->client->dev, "power_mitigate: already BPP\n");
		return;
	}

	/*
	 * Align supported only for mfg=0x72.
	 * NOTE: need to have this check for compat mode.
	 */
	if (charger->mfg != WLC_MFG_GOOGLE ||
	    !p9221_check_feature(charger, WLCF_DREAM_DEFEND)) {
		const char *txid = p9221_get_tx_id_str(charger);

		dev_info(&charger->client->dev,
			 "power_mitigate: not DD mfg=%x, id=%s\n",
			 charger->mfg, txid ? txid : "<>");
		return;
	}

	if (charger->fod_cnt < P9221_FOD_MAX_TIMES) {
		force_set_fod(charger);
		charger->fod_cnt++;
		dev_info(&charger->client->dev,
			 "power_mitigate: send FOD, cnt=%d\n",
			 charger->fod_cnt);
	} else {
		dev_info(&charger->client->dev,
			 "power_mitigate: power mitigation fail!\n");
		charger->fod_cnt = 0;
	}
}

static void p9221_init_align(struct p9221_charger_data *charger)
{
	const bool disabled = charger->prop_mode_en || charger->pdata->disable_align;

	/* Reset values used for alignment */
	charger->alignment_last = -1;
	charger->current_filtered = 0;
	charger->current_sample_cnt = 0;
	charger->mfg_check_count = 0;
	/* Disable misaligned message in high power mode, b/159066422 */
	if (!charger->online || disabled) {
		charger->align = WLC_ALIGN_CENTERED;
		return;
	}
	schedule_delayed_work(&charger->align_work,
			      msecs_to_jiffies(P9221_ALIGN_DELAY_MS));
}

static void p9xxx_align_check(struct p9221_charger_data *charger)
{
	const int align_delta = charger->align_delta != 0 ?
			charger->align_delta : charger->pdata->align_delta;
	int res, wlc_freq_threshold;
	u32 wlc_freq, current_scaling = 0, current_temp;

	current_temp = (charger->current_filtered > 100) ? (charger->current_filtered - 100) : 0;
	if (charger->current_filtered <= charger->pdata->alignment_current_threshold) {
		current_scaling =
			charger->pdata->alignment_scalar_low_current *
			current_temp / 10;
		wlc_freq_threshold =
			charger->pdata->alignment_offset_low_current +
			current_scaling;
	} else {
		current_scaling =
			charger->pdata->alignment_scalar_high_current *
			current_temp / 10;
		wlc_freq_threshold =
			charger->pdata->alignment_offset_high_current -
			current_scaling;
	}
	wlc_freq_threshold += align_delta;

	res = charger->chip_get_op_freq(charger, &wlc_freq);
	if (res != 0) {
		logbuffer_log(charger->log, "align: failed to read op_freq");
		return;
	}
	wlc_freq = P9221_KHZ_TO_HZ(wlc_freq);

	if (wlc_freq < wlc_freq_threshold)
		charger->alignment = 0;
	else
		charger->alignment = 100;

	if (charger->alignment != charger->alignment_last) {
		logbuffer_log(charger->log,
			      "align: freq_thres=%d, alignment=%i. op_freq=%u. current_avg=%u",
			      wlc_freq_threshold,
			      charger->alignment, wlc_freq,
			      charger->current_filtered);
		charger->alignment_last = charger->alignment;
		schedule_work(&charger->uevent_work);
	}
}

static void p9221_align_check(struct p9221_charger_data *charger,
			      u32 current_scaling)
{
	int res, align_buckets, i, wlc_freq_threshold, wlc_adj_freq;
	u32 wlc_freq;

	res = charger->chip_get_op_freq(charger, &wlc_freq);
	if (res != 0) {
		logbuffer_log(charger->log, "align: failed to read op_freq");
		return;
	}
	wlc_freq = P9221_KHZ_TO_HZ(wlc_freq);

	align_buckets = charger->pdata->nb_alignment_freq - 1;

	charger->alignment = -1;
	wlc_adj_freq = wlc_freq + current_scaling;

	if (wlc_adj_freq < charger->pdata->alignment_freq[0]) {
		logbuffer_log(charger->log, "align: freq below range");
		return;
	}

	for (i = 0; i < align_buckets; i += 1) {
		if ((wlc_adj_freq > charger->pdata->alignment_freq[i]) &&
		    (wlc_adj_freq <= charger->pdata->alignment_freq[i + 1])) {
			charger->alignment = (WLC_ALIGNMENT_MAX * i) /
					     (align_buckets - 1);
			break;
		}
	}

	if (i >= align_buckets) {
		logbuffer_log(charger->log, "align: freq above range");
		return;
	}

	if (charger->alignment == charger->alignment_last)
		return;

	/*
	 *  Frequency needs to be higher than frequency + hysteresis before
	 *  increasing alignment score.
	 */
	wlc_freq_threshold = charger->pdata->alignment_freq[i] +
			     charger->pdata->alignment_hysteresis;

	if ((charger->alignment < charger->alignment_last) ||
	    (wlc_adj_freq >= wlc_freq_threshold)) {
		logbuffer_log(charger->log,
			      "align: alignment=%i. op_freq=%u. current_avg=%u",
			      charger->alignment, wlc_freq,
			      charger->current_filtered);
		charger->alignment_last = charger->alignment;
		schedule_work(&charger->uevent_work);
	}
}

static void p9221_align_work(struct work_struct *work)
{
	int res;
	u16 current_filter_sample;
	u32 current_now;
	u32 current_scaling = 0;

	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, align_work.work);

	if ((charger->chip_id == P9221_CHIP_ID) &&
	    (charger->pdata->alignment_freq == NULL))
		return;

	/* reset the alignment value */
	charger->alignment = -1;

	/* b/159066422 Disable misaligned message in high power mode */
	if (!charger->online || charger->prop_mode_en == true) {
		charger->align = WLC_ALIGN_CENTERED;
		schedule_work(&charger->uevent_work);
		return;
	}

	/* take the align_ws, must be released */
	__pm_stay_awake(charger->align_ws);

	if (charger->alignment_capable == ALIGN_MFG_CHECKING) {
		charger->mfg_check_count += 1;

		res = p9xxx_chip_get_tx_mfg_code(charger, &charger->mfg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", res);
			goto align_again;
		}

		/* No mfg update. Will check again on next schedule */
		if (charger->mfg == 0)
			goto align_again;

		if ((charger->mfg != WLC_MFG_GOOGLE) ||
		    !p9221_is_epp(charger)) {
			logbuffer_log(charger->log,
				      "align: not align capable mfg: 0x%x",
				      charger->mfg);
			charger->alignment_capable = ALIGN_MFG_FAILED;
			goto align_end;
		}
		charger->alignment_capable = ALIGN_MFG_PASSED;
	}

	/* move to ALIGN_MFG_CHECKING or cache the value */
	if (!p9221_check_feature(charger, WLCF_DREAM_ALIGN))
		goto align_again;

	if (charger->pdata->alignment_scalar != 0) {
		res = charger->chip_get_iout(charger, &current_now);
		if (res != 0) {
			logbuffer_log(charger->log, "align: failed to read IOUT");
			current_now = 0;
		}

		current_filter_sample =
			charger->current_filtered / WLC_CURRENT_FILTER_LENGTH;

		if (charger->current_sample_cnt < WLC_CURRENT_FILTER_LENGTH)
			charger->current_sample_cnt++;
		else
			charger->current_filtered -= current_filter_sample;

		charger->current_filtered += (current_now / WLC_CURRENT_FILTER_LENGTH);
		if (charger->log_current_filtered)
			dev_info(&charger->client->dev, "current = %umA, avg_current = %umA\n",
				 current_now, charger->current_filtered);

		current_scaling = charger->pdata->alignment_scalar * charger->current_filtered;
	}

	if (charger->chip_id == P9221_CHIP_ID)
		p9221_align_check(charger, current_scaling);
	else
		p9xxx_align_check(charger);

	if (!charger->hda_tz_votable)
		charger->hda_tz_votable = gvotable_election_get_handle(VOTABLE_HDA_TZ);

	if (charger->hda_tz_votable) {
		const bool align_ok = charger->alignment < WLC_ALIGN_LOWER_LIMIT
				      || charger->alignment > WLC_ALIGN_UPPER_LIMIT;

		gvotable_cast_int_vote(charger->hda_tz_votable, P9221_ALIGN_VOTER,
			HDA_TZ_WLC_NOT_ALIGN, !align_ok);
	}

align_again:
	/*
	 *  NOTE: mfg may be zero due to race condition during boot. If the
	 *  mfg check continues to fail then mfg is not correct and we do not
	 *  reschedule align_work. Always reschedule if alignment_capable is 1.
	 *  Check 10 times if alignment_capble is still 0.
	 */

	if ((charger->mfg_check_count < MFG_CHK_COUNT_MAX) ||
	    (charger->alignment_capable == ALIGN_MFG_PASSED)) {

		/* release the align_ws before return*/
		__pm_relax(charger->align_ws);

		schedule_delayed_work(&charger->align_work,
				      msecs_to_jiffies(P9221_ALIGN_DELAY_MS));

		return;
	}

align_end:

	/* release the align_ws */
	__pm_relax(charger->align_ws);

	dev_info(&charger->client->dev, "align_work ended(mfg_check_count=%d)\n",
		 charger->mfg_check_count);
}

static const char *p9221_get_tx_id_str(struct p9221_charger_data *charger)
{
	int ret;

	if (!p9221_is_online(charger))
		return NULL;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return NULL;
	}
	pm_runtime_put_sync(charger->dev);

	if (p9221_is_epp(charger)) {
		ret = p9xxx_chip_get_tx_id(charger, &charger->tx_id);
		if (ret && ret != -ENOTSUPP)
			dev_err(&charger->client->dev,
				"Failed to read txid %d\n", ret);
	} else {
		/*
		 * If pp_buf_valid is true, we have received a serial
		 * number from the Tx, copy it to tx_id. (pp_buf_valid
		 * is left true here until we go offline as we may
		 * read this multiple times.)
		 */
		if (charger->pp_buf_valid &&
		    sizeof(charger->tx_id) <= P9221R5_MAX_PP_BUF_SIZE)
			memcpy(&charger->tx_id, &charger->pp_buf[1],
			       sizeof(charger->tx_id));
	}
	scnprintf(charger->tx_id_str, sizeof(charger->tx_id_str),
		  "%08x", charger->tx_id);
	return charger->tx_id_str;
}

/* call holding mutex_lock(&chg_fts->feat_lock); */
static int feature_cache_lookup_by_id(struct p9221_charger_feature *chg_fts, u64 id)
{
	/* FIXME: lookup */
	struct p9221_charger_feature_entry *entry = &chg_fts->entries[0];
	int idx;

	for (idx = 0; idx < chg_fts->num_entries; idx++) {
		if (entry->quickid == id)
			break;

		entry++;
	}

	if (chg_fts->num_entries > 0 && idx < chg_fts->num_entries)
		return idx;
	else
		return -1;
}

/* call holding mutex_lock(&chg_fts->feat_lock); */
static void feature_update_cache(struct p9221_charger_feature *chg_fts,
				 u64 id, u64 ft)
{
	struct p9221_charger_feature_entry *entry = &chg_fts->entries[0];
	struct p9221_charger_feature_entry *oldest_entry;
	u32 oldest;
	int idx;

	chg_fts->age++;

	oldest = 0;
	oldest_entry = entry;

	/* TODO: reimplemente in terms of feature_cache_lookup_by_id() */
	for (idx = 0; idx < chg_fts->num_entries; idx++) {
		u32 age = chg_fts->age - entry->last_use;

		if (entry->quickid == id)
			goto store;

		if (age > oldest) {
			oldest_entry = entry;
			oldest = age;
		}
		entry++;
	}

	/* Not found, space still left, entry points to free spot. */
	if (idx < P9XXX_CHARGER_FEATURE_CACHE_SIZE)
		chg_fts->num_entries++;
	else
		entry = oldest_entry;

store:
	pr_debug("%s: tx_id=%llx, ft=%llx\n", __func__, id, ft);

	entry->quickid = id;
	entry->features = ft;
	entry->last_use = chg_fts->age;
	if (entry->features == 0)
		entry->last_use = 0;
}

/* call holding mutex_lock(&chg_fts->feat_lock); */
static bool feature_cache_update_entry(struct p9221_charger_feature *chg_fts,
				       u64 id, u64 mask, u64 ft)
{
	struct p9221_charger_feature_entry *entry = &chg_fts->entries[0];
	bool updated = false;
	int index;

	index = feature_cache_lookup_by_id(chg_fts, id);
	if (index < 0) {
		/* add the new tx_id with ft to the feature cache */
		feature_update_cache(chg_fts, id, ft);
		return true;
	}

	pr_debug("%s: tx_id=%llx, mask=%llx ft=%llx\n", __func__, id, mask, ft);

	/* update the features using mask and ft */
	entry = &chg_fts->entries[index];
	if ((entry->features & mask) != ft) {
		entry->features |= (mask & ft);
		updated = true;
	}

	/* return true when features for id were actually updated  */
	return updated;
}

/* call holding mutex_lock(&chg_fts->feat_lock); */
static bool feature_cache_lookup_entry(struct p9221_charger_feature *chg_fts,
				       u64 id, u64 mask, u64 ft)
{
	struct p9221_charger_feature_entry *entry = &chg_fts->entries[0];
	int index;

	/* lookup id, true if found and feature&mask matches. */
	index = feature_cache_lookup_by_id(chg_fts, id);
	if (index >= 0) {
		entry = &chg_fts->entries[index];
		if ((entry->features & mask) == ft)
			return true;
	}

	return false;
}

/* call holding mutex_lock(&chg_fts->feat_lock); */
static bool feature_is_enabled(struct p9221_charger_feature *chg_fts,
			       u64 id, u64 ft)
{
	bool enabled = false;

	if (id)
		enabled = feature_cache_lookup_entry(chg_fts, id, ft, ft);
	if (!enabled)
		enabled = (chg_fts->session_features & ft) != 0;

	return enabled;
}

static bool p9221_check_feature(struct p9221_charger_data *charger, u64 ft)
{
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	bool supported = false;
	u32 tx_id = 0;
	u8 val;

	mutex_lock(&chg_fts->feat_lock);

	/*  txid is available sometime after placing the device on the charger */
	if (p9221_get_tx_id_str(charger) != NULL)
		tx_id = charger->tx_id;

	/* tx_ix = 0 will check only the session features */
	supported = feature_is_enabled(chg_fts, tx_id, ft);
	if (supported)
		goto unlock;

	if (!supported && !charger->pdata->feat_compat_mode)
		goto unlock;

	/* compat mode until the features API is usedm check txid */
	val = (tx_id & TXID_TYPE_MASK) >> TXID_TYPE_SHIFT;
	if (val == TXID_DD_TYPE || val == TXID_DD_TYPE2)
		supported = true;

	/* NOTE: some features need to be tied to mfgid */
	if (supported && charger->pdata->feat_compat_mode) {
		bool updated;

		updated = feature_cache_update_entry(chg_fts, tx_id, ft, ft);
		if (updated)
			pr_debug("%s: tx_id=%x, ft=%llx supported=%d\n", __func__,
				  tx_id, ft, supported);
	}
unlock:
	mutex_unlock(&chg_fts->feat_lock);
	return supported;
}


static int p9382_get_ptmc_id_str(char *buffer, int len,
				 struct p9221_charger_data *charger)
{
	int ret;
	uint16_t ptmc_id;

	if (!p9221_is_online(charger))
		return -ENODEV;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(charger->dev);

	ret = p9xxx_chip_get_tx_mfg_code(charger, &ptmc_id);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to read device prmc %d\n", ret);
		return ret;
	}

	return scnprintf(buffer, len, "%04x", ptmc_id);
}

/*
 * DC_SUSPEND is used to prevent inflow from wireless charging. When present
 * will return 1 if the user has disabled the source (override online).
 */
static int p9221_get_psy_online(struct p9221_charger_data *charger)
{
	int suspend = -EINVAL;
	int ret;
	u8 val;

	if (!charger->dc_suspend_votable)
		charger->dc_suspend_votable =
			gvotable_election_get_handle("DC_SUSPEND");
	if (charger->dc_suspend_votable)
		suspend = gvotable_get_current_int_vote(
				charger->dc_suspend_votable);

	/* TODO: not sure if this needs to be reported */
	if (suspend < 0)
		return suspend;
	if (suspend || !charger->online || !charger->enabled)
		return 0;

	ret = charger->chip_get_sys_mode(charger, &val);
	if (ret != 0) {
		dev_err(&charger->client->dev, "WLC online but cannot access i2c, ret=%d\n", ret);
		/* pm_stay_awake(charger->dev) is needed for schedule notifier_work */
		if (!delayed_work_pending(&charger->notifier_work)) {
			charger->check_dc = true;
			pm_stay_awake(charger->dev);
			if (!schedule_delayed_work(&charger->notifier_work,
						   msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS)))
				pm_relax(charger->dev);
		}
	}
	return charger->wlc_dc_enabled ? PPS_PSY_PROG_ONLINE : 1;
}

static int p9221_has_dc_in(struct p9221_charger_data *charger)
{
	union power_supply_propval prop;
	int ret;

	if (!charger->dc_psy)
		charger->dc_psy = power_supply_get_by_name("dc");
	if (!charger->dc_psy)
		return -EINVAL;

	ret = power_supply_get_property(charger->dc_psy,
					POWER_SUPPLY_PROP_PRESENT, &prop);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"Error getting charging status: %d\n", ret);
		return -EINVAL;
	}

	return prop.intval != 0;
}

static void p9221_charge_stats_init(struct p9221_charge_stats *chg_data)
{
	memset(chg_data, 0, sizeof(struct p9221_charge_stats));
	chg_data->cur_soc = -1;
}

static int p9221_stats_init_capabilities(struct p9221_charger_data *charger)
{
	struct p9221_charge_stats *chg_data = &charger->chg_data;
	const u8 ac_ver = 0;
	const u8 flags = 0;
	u8 sys_mode = 0;
	u16 ptmc_id = 0;
	int ret = 0;

	ret = p9xxx_chip_get_tx_mfg_code(charger, &ptmc_id);
	ret |= charger->chip_get_sys_mode(charger, &sys_mode);

	chg_data->adapter_capabilities[0] = flags << 8 | ac_ver;
	chg_data->adapter_capabilities[1] = ptmc_id;

	chg_data->receiver_state[0] = sys_mode;

	return ret ? -EIO : 0;
}

static int p9221_stats_update_state(struct p9221_charger_data *charger)
{
	struct p9221_charge_stats *chg_data = &charger->chg_data;
	u8 flags = 0;

	flags |= charger->prop_mode_en << 0;
	flags |= charger->is_mfg_google << 1;
	flags |= charger->wlc_dc_enabled << 2;

	chg_data->adapter_capabilities[0] |= flags << 8;

	return 0;
}

static void p9221_update_head_stats(struct p9221_charger_data *charger)
{
	u32 vout_mv, iout_ma;
	u32 wlc_freq = 0;
	u8 sys_mode;
	int ret;

	ret = charger->chip_get_sys_mode(charger, &sys_mode);
	if (ret != 0 || sys_mode <= 0)
		return;

	/* Only allow updates to higher system modes */
	if (sys_mode > charger->chg_data.adapter_type || sys_mode == P9XXX_SYS_OP_MODE_PROPRIETARY)
		charger->chg_data.adapter_type = sys_mode;

	ret = charger->chip_get_op_freq(charger, &wlc_freq);
	if (ret != 0)
		wlc_freq = -1;

	charger->chg_data.of_freq = wlc_freq;

	if (charger->wlc_dc_enabled) {
		ret = charger->chip_get_rx_ilim(charger, &iout_ma);
		if (ret)
			iout_ma = 0;
	} else if (!charger->dc_icl_votable) {
		iout_ma = 0;
	} else {
		int iout_ua;

		/* the unit charger->dc_icl_votable is uA, need to change to mA */
		iout_ua =
			gvotable_get_current_int_vote(charger->dc_icl_votable);
		if (iout_ua < 0)
			iout_ma = 0;
		else
			iout_ma = iout_ua / 1000;
	}
	charger->chg_data.cur_conf = iout_ma;

	if (charger->wlc_dc_enabled) {
		vout_mv = charger->pdata->max_vout_mv;
	} else if (p9221_ready_to_read(charger) < 0) {
		vout_mv = 0;
	} else {
		ret = charger->chip_get_vout_max(charger, &vout_mv);
		if (ret)
			vout_mv = 0;
	}
	charger->chg_data.volt_conf = vout_mv;
}

static void p9221_update_soc_stats(struct p9221_charger_data *charger,
				   int cur_soc)
{
	const ktime_t now = get_boot_sec();
	struct p9221_soc_data *soc_data;
	u32 vrect_mv, iout_ma, cur_pout;
	int ret, temp, interval_time = 0;
	u32 wlc_freq = 0;
	u8 sys_mode;

	ret = charger->chip_get_sys_mode(charger, &sys_mode);
	if (ret != 0)
		return;

	ret = charger->chip_get_op_freq(charger, &wlc_freq);
	if (ret != 0)
		wlc_freq = -1;

	ret = charger->chip_get_die_temp(charger, &temp);
	if (ret == 0)
		temp = P9221_MILLIC_TO_DECIC(temp);
	else
		temp = -1;

	ret = charger->chip_get_vrect(charger, &vrect_mv);
	if (ret != 0)
		vrect_mv = 0;

	ret = charger->chip_get_iout(charger, &iout_ma);
	if (ret != 0)
		iout_ma = 0;

	soc_data = &charger->chg_data.soc_data[cur_soc];

	soc_data->vrect = vrect_mv;
	soc_data->iout = iout_ma;
	soc_data->sys_mode = sys_mode;
	soc_data->die_temp = temp;
	soc_data->of_freq = wlc_freq;
	soc_data->alignment = charger->alignment;

	cur_pout = vrect_mv * iout_ma;
	if ((soc_data->pout_min == 0) || (soc_data->pout_min > cur_pout))
		soc_data->pout_min = cur_pout;
	if ((soc_data->pout_max == 0) || (soc_data->pout_max < cur_pout))
		soc_data->pout_max = cur_pout;

	if (soc_data->last_update != 0)
		interval_time = now - soc_data->last_update;

	soc_data->elapsed_time += interval_time;
	soc_data->pout_sum += cur_pout * interval_time;
	soc_data->last_update = now;
}

static void p9221_check_adapter_type(struct p9221_charger_data *charger)
{
	/*  txid is available sometime after placing the device on the charger */
	if (p9221_get_tx_id_str(charger) != NULL) {
		u8 id_type = (charger->tx_id & TXID_TYPE_MASK) >> TXID_TYPE_SHIFT;

		if (charger->chg_data.adapter_type < P9XXX_SYS_OP_MODE_PROPRIETARY &&
		    id_type != 0 && id_type != charger->chg_data.adapter_type) {
			pr_info("%s: tx_id=%08x, adapter_type=%x->%x\n", __func__,
				 charger->tx_id, charger->chg_data.adapter_type,
				 id_type);

			charger->chg_data.adapter_type = id_type;

			if (id_type == TXID_DD_TYPE2)
				charger->chg_data.start_time = get_boot_sec();
		}
	}
}

static int p9221_chg_data_head_dump(char *buff, int max_size,
				    const struct p9221_charge_stats *chg_data)
{
	return scnprintf(buff, max_size, "A:%d,%d,%d,%d,%d",
			 chg_data->adapter_type, chg_data->cur_soc,
			 chg_data->volt_conf, chg_data->cur_conf,
			 chg_data->of_freq);
}

static int p9221_adapter_capabilities_dump(char *buff, int max_size,
					   const struct p9221_charge_stats *chg_data)
{
	return scnprintf(buff, max_size, "D:%x,%x,%x,%x,%x, %x,%x",
			 chg_data->adapter_capabilities[0],
			 chg_data->adapter_capabilities[1],
			 chg_data->adapter_capabilities[2],
			 chg_data->adapter_capabilities[3],
			 chg_data->adapter_capabilities[4],
			 chg_data->receiver_state[0],
			 chg_data->receiver_state[1]);
}

static int p9221_soc_data_dump(char *buff, int max_size,
			       const struct p9221_charge_stats *chg_data,
			       int index)
{
	return scnprintf(buff, max_size, "%d:%d, %d,%ld,%d, %d,%d, %d,%d,%d,%d",
			 index,
			 chg_data->soc_data[index].elapsed_time,
			 chg_data->soc_data[index].pout_min / 100000,
			 chg_data->soc_data[index].pout_sum /
			 chg_data->soc_data[index].elapsed_time/ 100000,
			 chg_data->soc_data[index].pout_max / 100000,
			 chg_data->soc_data[index].of_freq,
			 chg_data->soc_data[index].alignment,
			 chg_data->soc_data[index].vrect,
			 chg_data->soc_data[index].iout,
			 chg_data->soc_data[index].die_temp,
			 chg_data->soc_data[index].sys_mode);
}

/* needs mutex_lock(&charger->stats_lock); */
static void p9221_dump_charge_stats(struct p9221_charger_data *charger)
{
	char buff[128];
	int i = 0;

	if (charger->chg_data.cur_soc < 0)
		return;

	/* Dump the head */
	p9221_chg_data_head_dump(buff, sizeof(buff), &charger->chg_data);
	logbuffer_log(charger->log, "%s", buff);

	/* Dump the adapter capabilities */
	p9221_adapter_capabilities_dump(buff, sizeof(buff), &charger->chg_data);
	logbuffer_log(charger->log, "%s", buff);


	for (i = 0; i < WLC_SOC_STATS_LEN; i++) {
		if (charger->chg_data.soc_data[i].elapsed_time == 0)
			continue;
		p9221_soc_data_dump(buff, sizeof(buff), &charger->chg_data, i);
		logbuffer_log(charger->log, "%s", buff);
	}
}

static int p9221_get_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	int rc, ret = 0;
	u32 temp;

	switch (prop) {
	/* check for field */
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = p9221_has_dc_in(charger);
		if (val->intval < 0)
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (charger->wait_for_online) {
			val->intval = 1;
			break;
		}

		val->intval = p9221_get_psy_online(charger);
		/* thermal throttled + PDET detected + not in RTX mode */
		if (val->intval)
			break;
		val->intval = charger->online_spoof && charger->ben_state == 0;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		/* val->strval == NULL means NODATA */
		val->strval = p9221_get_tx_id_str(charger);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Zero may be returned on transition to wireless "online", as
		 * last_capacity is reset to -1 until capacity is re-written
		 * from userspace, leading to a new csp packet being sent.
		 *
		 * b/80435107 for additional context
		 */
		if (charger->last_capacity > 0)
			val->intval = charger->last_capacity;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (charger->wlc_dc_enabled) {
			rc = charger->chip_get_rx_ilim(charger, &temp);
			if (rc == 0)
				charger->wlc_dc_current_now = temp * 1000; /* mA to uA */

			val->intval = rc ? : charger->wlc_dc_current_now;
		} else {
			if (!charger->dc_icl_votable) {
				val->intval = -EAGAIN;
				break;
			}
			val->intval = gvotable_get_current_int_vote(
						charger->dc_icl_votable);
		}
		break;
#ifdef CONFIG_QC_COMPAT
	case POWER_SUPPLY_PROP_AICL_DELAY:
		val->intval = charger->aicl_delay_ms;
		break;
	case POWER_SUPPLY_PROP_AICL_ICL:
		val->intval = charger->aicl_icl_ua;
		break;
#endif
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = p9221_ready_to_read(charger);
		if (rc < 0) {
			val->intval = 0;
		} else {
			rc = charger->chip_get_iout(charger, &temp);
			if (charger->wlc_dc_enabled && (rc == 0))
				charger->wlc_dc_current_now = temp * 1000; /* mA to uA */

			val->intval = rc ? : temp * 1000; /* mA to uA */
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = p9221_ready_to_read(charger);
		if (rc < 0) {
			val->intval = 0;
		} else if (charger->wlc_dc_enabled) {
			rc = charger->chip_get_vout(charger, &temp);
			if (rc == 0)
				charger->wlc_dc_voltage_now = temp * 1000; /* mV to uV */
		} else {
			rc = charger->chip_get_vout(charger, &temp);
		}

		val->intval = rc ? : temp * 1000; /* mV to uV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (charger->wlc_dc_enabled) {
			val->intval = charger->pdata->max_vout_mv * 1000; /* mV to uV */
		} else {
			rc = p9221_ready_to_read(charger);
			if (rc < 0) {
				val->intval = 0;
			} else {
				u32 mv;

				rc = charger->chip_get_vout_max(charger, &mv);
				if (rc)
					val->intval = 0;
				else
					val->intval = mv * 1000; /* mv to uV */
			}
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = 5000000; // TODO: fix it
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = p9221_ready_to_read(charger);
		if (!rc) {
			rc = charger->chip_get_die_temp(charger, &val->intval);
			if (!rc)
				val->intval = P9221_MILLIC_TO_DECIC(val->intval);
		}

		if (rc)
			val->intval = 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't get prop %d, ret=%d\n", prop, ret);
	return ret;
}

#define P9221_CHARGE_STATS_DELAY_SEC 10
static void p9221_charge_stats_hda_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, charge_stats_hda_work.work);
	struct p9221_charge_stats *chg_data = &charger->chg_data;
	const ktime_t now = get_boot_sec();
	const ktime_t start_time = chg_data->start_time;
	const ktime_t elap = now - chg_data->start_time;
	int tz_vote = HDA_TZ_NONE;

	mutex_lock(&charger->stats_lock);

	if (charger->online == 0 || charger->last_capacity < 0 || charger->last_capacity > 100)
		goto unlock_done;

	/* Charge_stats buffer is Empty */
	if (chg_data->cur_soc < 0) {
		/* update the head */
		chg_data->cur_soc = charger->last_capacity;
		chg_data->last_soc = charger->last_capacity;
		p9221_update_head_stats(charger);
		chg_data->start_time = get_boot_sec();
	} else if (start_time && elap > P9221_CHARGE_STATS_DELAY_SEC) {
		/*
		 * Voltage, Current and sys_mode are not correct
		 * on wlc start. Debounce by 10 seconds.
		 */
		p9221_update_head_stats(charger);
		p9221_stats_init_capabilities(charger);
		chg_data->start_time = 0;
	}

	p9221_check_adapter_type(charger);
	p9221_stats_update_state(charger);

	if (!charger->hda_tz_votable)
		charger->hda_tz_votable = gvotable_election_get_handle(VOTABLE_HDA_TZ);

	if (charger->chg_data.adapter_type == TXID_DD_TYPE)
		tz_vote = HDA_TZ_WLC_EPP_1P;
	else if (charger->chg_data.adapter_type == TXID_DD_TYPE2)
		tz_vote = HDA_TZ_WLC_EPP_L7;

	if (charger->hda_tz_votable && ((charger->chg_data.adapter_type == TXID_DD_TYPE)
	    || (charger->chg_data.adapter_type == TXID_DD_TYPE2)))
		gvotable_cast_int_vote(charger->hda_tz_votable, P9221_WLC_VOTER,
					tz_vote, tz_vote);

	/* SOC changed, store data to the last one. */
	if (chg_data->last_soc != charger->last_capacity)
		p9221_update_soc_stats(charger, chg_data->last_soc);
	/* update currect_soc data */
	p9221_update_soc_stats(charger, charger->last_capacity);

	chg_data->last_soc = charger->last_capacity;

	schedule_delayed_work(&charger->charge_stats_hda_work,
			      msecs_to_jiffies(P9221_CHARGE_STATS_TIMEOUT_MS));

unlock_done:
	mutex_unlock(&charger->stats_lock);
}

static bool feature_check_fast_charge(struct p9221_charger_data *charger)
{
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	bool enabled;

	mutex_lock(&chg_fts->feat_lock);

	/* ignore the feature in compatibility until the first vote */
	if (charger->pdata->feat_compat_mode) {
		pr_debug("%s: COMPAT FAST_CHARGE ENABLED\n", __func__);
		mutex_unlock(&chg_fts->feat_lock);
		return true;
	}

	enabled = feature_is_enabled(chg_fts, 0, WLCF_FAST_CHARGE);
	if (enabled)
		pr_debug("%s: feature enabled=%d\n", __func__, enabled);

	mutex_unlock(&chg_fts->feat_lock);

	return enabled;
}

static int p9221_set_hpp_dc_icl(struct p9221_charger_data *charger, bool enable)
{
	int ret;

	if (!charger->dc_icl_votable)
		return 0;

	if (charger->pdata->has_sw_ramp && enable) {
		dev_dbg(&charger->client->dev, "%s: voter=%s, icl=%d\n",
			__func__, HPP_DC_ICL_VOTER, P9221_DC_ICL_HPP_UA);
		ret = p9xxx_sw_ramp_icl(charger, P9221_DC_ICL_HPP_UA);
		if (ret == 0)
			ret = gvotable_cast_long_vote(charger->dc_icl_votable,
						      HPP_DC_ICL_VOTER,
						      P9221_DC_ICL_HPP_UA,
						      enable);
		if (ret == 0)
			ret = gvotable_cast_int_vote(charger->dc_icl_votable,
						     P9221_RAMP_VOTER, 0, false);
		return ret;
	}

	return gvotable_cast_long_vote(charger->dc_icl_votable,
				       HPP_DC_ICL_VOTER,
				       enable ? P9221_DC_ICL_HPP_UA : 0,
				       enable);
}

/*
 * Auth delay must be there for EPP until the timeout since we don't know
 * if the TX supports WPC1.3 Auth until the timeout expires (or until
 * set features tell us the auth limit is not needed anymore)
 * hold mutex_lock(&charger->auth_lock);
 */
int p9221_set_auth_dc_icl(struct p9221_charger_data *charger, bool enable)
{
	int ret = 0, icl_ua;

	icl_ua = enable ? P9221_AUTH_DC_ICL_UA_500 : 0;

	if (!charger->dc_icl_votable) {
		charger->dc_icl_votable = gvotable_election_get_handle("DC_ICL");
		if (!charger->dc_icl_votable) {
			dev_err(&charger->client->dev,
				"Could not get votable: DC_ICL\n");
			goto exit;
		}
	}

	if (enable && !charger->auth_delay) {
		dev_info(&charger->client->dev, "Enable Auth ICL (%d)\n", ret);
		charger->auth_delay = true;
	} else if (!enable && charger->auth_delay) {
		dev_info(&charger->client->dev, "Disable Auth ICL (%d)\n", ret);
		charger->auth_delay = false;
		cancel_delayed_work(&charger->auth_dc_icl_work);
		alarm_try_to_cancel(&charger->auth_dc_icl_alarm);
	}

	if (enable)
		ret = p9xxx_sw_ramp_icl(charger, icl_ua);

	if (ret == 0)
		ret = gvotable_cast_int_vote(charger->dc_icl_votable, AUTH_DC_ICL_VOTER,
					     icl_ua, enable);
	if (ret == 0)
		ret = gvotable_cast_int_vote(charger->dc_icl_votable,
					     P9221_RAMP_VOTER, 0, false);
	if (ret < 0)
		goto exit;

	if (!charger->csi_status_votable)
		charger->csi_status_votable =
				gvotable_election_get_handle(VOTABLE_CSI_STATUS);
	if (charger->csi_status_votable)
		gvotable_cast_long_vote(charger->csi_status_votable,
					"CSI_STATUS_ADA_AUTH",
					CSI_STATUS_Adapter_Auth,
					charger->auth_delay);

exit:
	return ret;
}

static int p9xxx_check_alignment(struct p9221_charger_data *charger)
{
	int ret = 0;

	if (charger->pdata->needs_align_check == 0)
		return ret;

	if (charger->alignment == 100) {
		dev_dbg(&charger->client->dev, "Alignment check OK\n");
	} else if (charger->alignment == -1 && charger->mfg_check_count < MFG_CHK_COUNT_MAX) {
		ret = -EAGAIN;
		dev_dbg(&charger->client->dev, "Alignment checking\n");
	} else {
		ret = -EOPNOTSUPP;
		dev_err(&charger->client->dev, "Misalignment!\n");
	}

	return ret;
}

static int p9xxx_check_fast_charge(struct p9221_charger_data *charger)
{
	const bool feat_enable = feature_check_fast_charge(charger);

	mutex_lock(&charger->auth_lock);

	if (feat_enable) {
		pr_debug("%s: Feature check OK\n", __func__);
		mutex_unlock(&charger->auth_lock);
		return 0;
	}

	if (charger->auth_delay || !charger->set_auth_icl) {
		dev_info(&charger->client->dev, "Auth delay\n");
		mutex_unlock(&charger->auth_lock);
		return -EAGAIN;
	}

	dev_warn(&charger->client->dev, "Feature check failed\n");
	p9221_set_auth_dc_icl(charger, false);
	mutex_unlock(&charger->auth_lock);
	return -EOPNOTSUPP;
}

/*
 * return 1          : in WLC-DC mode
 * return -EAGAIN    : retry due to not ready
 * return -EOPNOTSUPP: WLC-DC not support, WLC offline
 */
static int p9221_enable_wlc_dc(struct p9221_charger_data *charger)
{
	const u32 req_pwr = charger->de_hpp_neg_pwr > 0 ?
			    charger->de_hpp_neg_pwr : charger->pdata->hpp_neg_pwr;
	const int extben_gpio = charger->pdata->ext_ben_gpio;
	int ret;
	u8 val8;

	/* not there, must return not supp */
	if (!charger->pdata->has_wlc_dc || !p9221_is_online(charger) || !p9221_is_epp(charger))
		return -EOPNOTSUPP;

	/* prevent 3p charger for compat mode */
	if (charger->mfg == 0) {
		dev_warn(&charger->client->dev, "mfg not ready for HPP\n");
		return -EAGAIN;
	}

	if (!charger->is_mfg_google) {
		dev_warn(&charger->client->dev, "HPP not allowed, mfg: 0x%x\n", charger->mfg);
		return -EOPNOTSUPP;
	}

	/*
	 * Lower DC_ICL for HPP to improve communications while
	 * the charger ramps up the voltage to the limit.
	 */
	ret = p9221_set_hpp_dc_icl(charger, true);
	if (ret < 0)
		dev_warn(&charger->client->dev, "cannot set HPP DC ICL: %d\n", ret);

	if (charger->last_capacity < 0) {
		schedule_delayed_work(&charger->soc_work, 0);
		dev_dbg(&charger->client->dev, "retry, last_capacity=%d\n",
			charger->last_capacity);
		return -EAGAIN;
	}
	if (charger->last_capacity > WLC_HPP_SOC_LIMIT)
		goto not_hpp;


	/* need to check calibration is done before re-negotiate */
	if (!charger->chip_is_calibrated(charger)) {
		dev_warn(&charger->client->dev, "Calibrating\n");
		return -EAGAIN;
	}

	ret = charger->reg_read_8(charger, P9221R5_EPP_TX_GUARANTEED_POWER_REG, &val8);
	if (ret < 0 || val8 < P9XXX_TX_GUAR_PWR_15W) {
		dev_warn(&charger->client->dev, "Tx guar_pwr=%dW\n", ret == 0 ? val8 / 2 : ret);
		goto not_hpp;
	}

	ret = p9xxx_check_fast_charge(charger);
	if (ret == -EAGAIN)
		return ret;
	if (ret == -EOPNOTSUPP)
		goto not_hpp;

	/* Check alignment before enabling proprietary mode */
	ret = p9xxx_check_alignment(charger);
	if (ret == -EAGAIN)
		return ret;
	if (ret == -EOPNOTSUPP)
		goto not_hpp;

	if (charger->prop_mode_en && p9xxx_is_capdiv_en(charger))
		return 1;
	/*
	 * run ->chip_prop_mode_en() if proprietary mode or cap divider
	 * mode isn't enabled (i.e. with p9412_prop_mode_enable())
	 *
	 * return 0      : WLC-DC is not enabled
	 * return 1      : success enter WLC-DC
	 * return -ENODEV: WLC is offline
	 */
	ret = set_renego_state(charger, P9XXX_ENABLE_PROPMODE);
	if (ret == -EAGAIN) {
		dev_dbg(&charger->client->dev, "Set renego state retry\n");
		return ret;
	}
	charger->chip_set_ovp(charger, OVSET_HPP);
	ret = charger->chip_prop_mode_en(charger, req_pwr);
	if (ret == -ENODEV) {
		dev_err(&charger->client->dev, "Offline during PROP Mode\n");
		goto not_hpp;
	}
	set_renego_state(charger, P9XXX_AVAILABLE);

	/* Only returns 1 for success */
	if (ret != 1) {
		ret = p9221_reset_wlc_dc(charger);
		if (ret < 0)
			dev_err(&charger->client->dev, "%s: HPP not supported\n", __func__);
		return -EOPNOTSUPP;
	}

	p9221_write_fod(charger);

	charger->wlc_dc_enabled = true;

	if (extben_gpio)
		p9xxx_gpio_set_value(charger, extben_gpio, 1);

	p9221_set_switch_reg(charger, true);

	ret = p9xxx_chip_set_cmfet_reg(charger, charger->wlc_dc_comcap);
	if (ret < 0 && ret != -ENOTSUPP)
		dev_warn(&charger->client->dev, "Fail to set comm cap(%d)\n", ret);

	return 1;
not_hpp:
	ret = p9221_set_hpp_dc_icl(charger, false);
	if (ret < 0)
		dev_warn(&charger->client->dev, "Cannot disable HPP_ICL (%d)\n", ret);
	return -EOPNOTSUPP;
}

/* < 0 error, 0 = no changes, 1 = changed */
static int p9221_set_psy_online(struct p9221_charger_data *charger, int online)
{
	const bool wlc_dc_enabled = charger->wlc_dc_enabled;
	const bool enabled = charger->enabled;
	int ret;

	if (online < 0 || online > PPS_PSY_PROG_ONLINE)
		return -EINVAL;

	pr_info("%s: online=%d, enabled=%d wlc_dc_enabled=%d prop_mode_en=%d\n",
		__func__, online, enabled, wlc_dc_enabled,
		charger->prop_mode_en);
	/* online = 2 enable LL, return < 0 if NOT on LL */
	if (online == PPS_PSY_PROG_ONLINE) {
		const bool feat_enable = feature_check_fast_charge(charger);

		if (!enabled) {
			dev_warn(&charger->client->dev,
				 "Cannot send PROG with enable=%d, wlc_dc_enabled=%d\n",
				 enabled, wlc_dc_enabled);
			if (wlc_dc_enabled)
				p9221_reset_wlc_dc(charger);
			return -EINVAL;
		}

		/* Ping? */
		if (wlc_dc_enabled) {
			/* TODO: disable fast charge if enabled */
			if (feat_enable == 0)
				pr_debug("%s: FAST_CHARGE disabled\n", __func__);

			return 0;
		}

		/*
		 * p9221_enable_wlc_dc()
		 * return 1          : in WLC-DC mode
		 * return -EAGAIN    : retry due to not ready
		 * return -EOPNOTSUPP: WLC-DC not support, WLC offline
		 */
		ret = p9221_enable_wlc_dc(charger);
		return ret;
	}
	if (wlc_dc_enabled) {
		/* TODO: thermals might come in and disable with 0 */
		p9221_reset_wlc_dc(charger);
	}

	/*
	 * Asserting the enable line will automatically take bring
	 * us online if we are in field.  De-asserting the enable
	 * line will automatically take us offline if we are in field.
	 * This is due to the fact that DC in will change state
	 * appropriately when we change the state of this line.
	 */
	charger->enabled = !!online;

	ret = p9221_set_hpp_dc_icl(charger, false);
	if (ret < 0)
		dev_warn(&charger->client->dev, "Cannot disable HPP_ICL (%d)\n", ret);

	gvotable_cast_int_vote(charger->dc_icl_votable,
				P9221_HPP_VOTER, 0, false);

	dev_warn(&charger->client->dev, "Set enable %d, wlc_dc_enabled:%d->%d\n",
		charger->enabled, wlc_dc_enabled, charger->wlc_dc_enabled);

	gvotable_cast_bool_vote(charger->wlc_disable_votable,
				P9221_WLC_VOTER, !charger->enabled);

	return 1;
}

/* 400 seconds debounce for auth per WPC spec */
#define DREAM_DEBOUNCE_TIME_S 400

/* trigger DD */
static void p9221_dream_defend(struct p9221_charger_data *charger)
{
	struct gvotable_election *csi_type_votable = charger->csi_type_votable;
	const ktime_t now = get_boot_sec();
	bool is_ac = false;
	u32 threshold;
	int ret;

	if (!charger->trigger_dd)
		charger->trigger_dd = DREAM_DEBOUNCE_TIME_S;

	/* debounce for auth TODO: use charger->auth_delay */
	if (now - charger->online_at < charger->trigger_dd) {
		pr_debug("%s: now=%lld, online_at=%lld delta=%lld\n", __func__,
			 now, charger->online_at, now - charger->online_at);
		return;
	}

	if (!charger->dc_avail_votable)
		charger->dc_avail_votable = gvotable_election_get_handle(VOTABLE_DC_AVAIL);
	if (!csi_type_votable)
		charger->csi_type_votable = gvotable_election_get_handle(VOTABLE_CSI_TYPE);
	if (csi_type_votable)
		is_ac = gvotable_get_current_int_vote(csi_type_votable) == CSI_TYPE_Adaptive;

	/* extended trigger soc for TTF calculations to ensure enough time for AC */
	if (!is_ac)
		charger->pdata->power_mitigate_ac_threshold = 0;
	else if (charger->pdata->power_mitigate_ac_threshold == 0)
		charger->pdata->power_mitigate_ac_threshold = charger->last_capacity + 1;

	if (charger->mitigate_threshold > 0)
		threshold = charger->mitigate_threshold;
	else if (charger->pdata->power_mitigate_threshold > 0 &&
		 charger->pdata->power_mitigate_ac_threshold > 0)
		threshold = charger->pdata->power_mitigate_ac_threshold;
	else
		threshold = charger->pdata->power_mitigate_threshold;

	pr_debug("dream_defend soc:%d threshold:%d\n", charger->last_capacity, threshold);

	if (!threshold)
		return;

	if (charger->last_capacity < 0) {
		schedule_delayed_work(&charger->soc_work, 0);
		dev_info(&charger->client->dev, "retry, last_capacity=%d\n",
			 charger->last_capacity);
		return;
	}

	if (charger->last_capacity > threshold &&
		!charger->trigger_power_mitigation) {
		if (charger->dc_avail_votable)
			gvotable_cast_int_vote(charger->dc_avail_votable, DD_VOTER, 0, true);
		if (charger->prop_mode_en)
			return;
		/* trigger_power_mitigation is the same as dream defend */
		charger->trigger_power_mitigation = true;
		ret = delayed_work_pending(&charger->power_mitigation_work);
		if (!ret)
			schedule_delayed_work(&charger->power_mitigation_work,
			    msecs_to_jiffies(P9221_POWER_MITIGATE_DELAY_MS));
	}

}

/*
 * identify TXID_DD_TYPE2 checking PTCPM and ID
 * return -EAGAIN until the id is available, 0 on non match
 */
static int p9221_ll_check_id(struct p9221_charger_data *charger)
{
	uint16_t ptmc_id = charger->mfg;
	int ret;

	/*
	 * TODO: this returns 0 on 3rd party. Solve by debounce changes of
	 * last_capacity for some time after online and/or adding a robust
	 * way to detect if the device is actuall on a pad that needs the
	 * WAR and behaviors controlled by this check.
	 */
	if (ptmc_id == 0) {
		ret = p9xxx_chip_get_tx_mfg_code(charger, &ptmc_id);
		if (ret < 0 || ptmc_id == 0) {
			pr_debug("%s: cannot get mfg code ptmc_id=%x (%d)\n",
				 __func__, ptmc_id, ret);
			return -EAGAIN;
		}
	}

	if (ptmc_id != WLC_MFG_GOOGLE) {
		pr_debug("%s: ptmc_id=%x\n", __func__, ptmc_id);
		return 0;
	}

	/* NOTE: will keep the alternate limit and keep checking on 3rd party */
	if (p9221_get_tx_id_str(charger) == NULL) {
		pr_debug("%s: retry %x\n", __func__, charger->tx_id);
		return -EAGAIN;
	}

	/* optional check for P9221R5_EPP_TX_GUARANTEED_POWER_REG == 0x1e */

	pr_debug("%s: tx_ix=%08x\n", __func__, charger->tx_id);
	return ((charger->tx_id & TXID_TYPE_MASK) >> TXID_TYPE_SHIFT) == TXID_DD_TYPE2;
}

static void p9221_set_capacity(struct p9221_charger_data *charger, int capacity)
{
	int ret;

	mutex_lock(&charger->stats_lock);

	if (charger->last_capacity == capacity &&
		(capacity != 100 || !p9221_is_epp(charger)))
		goto unlock_done;

	charger->last_capacity = (capacity <= 100) ? capacity : 100;

	if (!p9221_is_online(charger))
		goto unlock_done;

	ret = p9221_send_csp(charger, charger->last_capacity);
	if (ret)
		dev_err(&charger->client->dev, "Could not send csp: %d\n", ret);

	/* p9221_is_online() is true when the device in TX mode. */
	if (charger->online && charger->last_capacity >= 0 && charger->last_capacity <= 100)
		mod_delayed_work(system_wq, &charger->charge_stats_hda_work, 0);

	if (charger->pdata->gpp_enhanced && charger->last_capacity > WLC_HPP_SOC_LIMIT)
		feature_gpp_15w_enable(charger, false);

	if (charger->online && p9221_is_epp(charger))
		p9221_dream_defend(charger);

	p9221_wcin_inlim(charger);

unlock_done:
	mutex_unlock(&charger->stats_lock);
}

static int check_hpp_fod_level(struct p9221_charger_data *charger,
			       int dc_voltage)
{
	const int hpp_fod_buckets = charger->pdata->nb_hpp_fod_vol;
	int i, hysteresis;

	for (i = 0; i < hpp_fod_buckets; i += 1) {
		if (i == charger->hpp_fod_level)
			hysteresis = 100000;
		else if (i == charger->hpp_fod_level - 1)
			hysteresis = -100000;
		else
			hysteresis = 0;

		if (dc_voltage < charger->pdata->hpp_fod_vol[i] + hysteresis)
			return i;
	}
	return 0;
}

static int p9221_set_property(struct power_supply *psy,
			      enum power_supply_property prop,
			      const union power_supply_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	bool changed = false;
	int rc, ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		rc = p9221_set_psy_online(charger, val->intval);
		if (rc < 0) {
			ret = rc;
			break;
		}
		changed = !!rc;
		break;
	case POWER_SUPPLY_PROP_CAPACITY: {
		int capacity = val->intval;

		/* TODO: ignore the direct calls when fuel-gauge is defined */
		if (charger->batt_psy && (capacity != 101)) {
			rc = p9221_capacity_raw(charger);
			if (rc >= 0 && capacity != rc) {
				pr_debug("%s: orig=%d new=%d\n", __func__,
					 val->intval, rc);
				capacity = rc;
			}
		}

		p9221_set_capacity(charger, capacity);
		break;
	}
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (val->intval < 0) {
			ret = -EINVAL;
			break;
		}

		if (!charger->dc_icl_votable) {
			ret = -EAGAIN;
			break;
		}

		ret = gvotable_cast_int_vote(charger->dc_icl_votable,
					     P9221_USER_VOTER,
					     val->intval, true);
		changed = true;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = charger->chip_set_vout_max(charger, P9221_UV_TO_MV(val->intval));
		/* this is extra, please verify */
		if (ret == 0)
			changed = true;
		break;

	/* route to p9412 if for wlc_dc */
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!charger->wlc_dc_enabled) {
			dev_dbg(&charger->client->dev,
				"Not WLC-DC, not allow to set dc current\n");
			ret = -EINVAL;
			break;
		}
		/* uA */
		charger->wlc_dc_current_now = val->intval;
		ret = charger->chip_set_rx_ilim(charger, P9221_UA_TO_MA(charger->wlc_dc_current_now));
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (!charger->wlc_dc_enabled) {
			charger->hpp_fod_level = 0;
			dev_dbg(&charger->client->dev,
				"Not WLC-DC, not allow to set Vout\n");
			ret = -EINVAL;
			break;
		}
		/* uV */
		charger->wlc_dc_voltage_now = val->intval;
		ret = charger->chip_set_vout_max(charger,
						 P9221_UV_TO_MV(charger->wlc_dc_voltage_now));
		if (ret < 0)
			break;
		p9221_write_fod(charger);
		break;

	default:
		return -EINVAL;
	}

	if (ret)
		dev_dbg(&charger->client->dev,
			"Couldn't set prop %d, ret=%d\n", prop, ret);

	if (changed)
		power_supply_changed(psy);

	return ret;
}

static int p9221_prop_is_writeable(struct power_supply *psy,
				   enum power_supply_property prop)
{
	int writeable = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		writeable = 1;
		break;
	default:
		break;
	}

	return writeable;
}

static int p9221_gbms_get_property(struct power_supply *psy,
				   enum gbms_property prop,
				   union gbms_propval *val)
{
	struct p9221_charger_data *charger = power_supply_get_drvdata(psy);
	int rc, ret = 0;
	u32 temp;

	switch (prop) {
	case GBMS_PROP_WLC_OP_FREQ:
		rc = p9221_ready_to_read(charger);
		if (!rc) {
			rc = charger->chip_get_op_freq(charger, &temp);
			if (!rc)
				val->prop.intval = P9221_KHZ_TO_HZ(temp);
		}
		if (rc)
			val->prop.intval = 0;
		break;
	case GBMS_PROP_WLC_VRECT:
		rc = p9221_ready_to_read(charger);
		if (!rc) {
			rc = charger->chip_get_vrect(charger, &temp);
			if (!rc)
				val->prop.intval = P9221_MV_TO_UV(temp);
		}
		if (rc)
			val->prop.intval = 0;
		break;
	case GBMS_PROP_WLC_VCPOUT:
		rc = p9221_ready_to_read(charger);
		if (!rc) {
			rc = charger->chip_get_vcpout(charger, &temp);
			if (!rc)
				val->prop.intval = P9221_MV_TO_UV(temp);
		}
		if (rc)
			val->prop.intval = 0;
		break;

	default:
		pr_debug("%s: route to p9221_get_property, psp:%d\n", __func__, prop);
		ret = -ENODATA;
		break;
	}

	return ret;
}

static int p9221_gbms_set_property(struct power_supply *psy,
				   enum gbms_property prop,
				   const union gbms_propval *val)
{
	pr_debug("%s: route to p9221_set_property, psp:%d\n", __func__, prop);
	return -ENODATA;
}

static int p9221_gbms_prop_is_writeable(struct power_supply *psy,
					enum gbms_property prop)
{
	int writeable = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		writeable = 1;
		break;
	default:
		break;
	}

	return writeable;
}

/* pm_stay_awake(charger->dev) is needed for schedule notifier_work */
static int p9221_notifier_cb(struct notifier_block *nb, unsigned long event,
			     void *data)
{
	struct power_supply *psy = data;
	struct p9221_charger_data *charger =
		container_of(nb, struct p9221_charger_data, nb);
	const char *batt_name = IS_ERR_OR_NULL(charger->batt_psy) || !charger->batt_psy->desc ?
				NULL : charger->batt_psy->desc->name;

	if (charger->ben_state)
		goto out;

	if (event != PSY_EVENT_PROP_CHANGED)
		goto out;

	if (strcmp(psy->desc->name, "dc") == 0) {
		charger->dc_psy = psy;
		charger->check_dc = true;
	} else if (strcmp(psy->desc->name, "dc-mains") == 0) {
		charger->check_dc = true;
	} else if (batt_name && strcmp(psy->desc->name, batt_name) == 0) {
		schedule_delayed_work(&charger->soc_work, 0);
	}

	if (!charger->check_dc)
		goto out;

	pm_stay_awake(charger->dev);

	if (!schedule_delayed_work(&charger->notifier_work,
				   msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS)))
		pm_relax(charger->dev);

out:
	return NOTIFY_OK;
}

static int p9221_clear_interrupts(struct p9221_charger_data *charger, u16 mask)
{
	int ret;

	mutex_lock(&charger->cmd_lock);

	ret = p9221_reg_write_16(charger, P9221R5_INT_CLEAR_REG, mask);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	ret = charger->chip_set_cmd(charger, P9221_COM_CLEAR_INT_MASK);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to reset INT: %d\n", ret);
	}
out:
	mutex_unlock(&charger->cmd_lock);
	return ret;
}

/*
 * Enable interrupts on the P9221, note we don't really need to disable
 * interrupts since when the device goes out of field, the P9221 is reset.
 */
static int p9221_enable_interrupts(struct p9221_charger_data *charger)
{
	u16 mask;
	int ret;

	dev_dbg(&charger->client->dev, "Enable interrupts\n");

	if (charger->ben_state) {
		/* enable necessary INT for RTx mode */
		mask = charger->ints.stat_rtx_mask;
	} else {
		mask = charger->ints.stat_limit_mask | charger->ints.stat_cc_mask |
		       charger->ints.vrecton_bit | charger->ints.prop_mode_mask |
		       charger->ints.extended_mode_bit | charger->ints.vout_changed_bit;

		if (charger->pdata->needs_dcin_reset ==
						P9221_WC_DC_RESET_MODECHANGED)
			mask |= charger->ints.mode_changed_bit;
	}
	ret = p9221_clear_interrupts(charger, P9XXX_INT_CLEAR_MASK);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not clear interrupts: %d\n", ret);

	ret = p9221_reg_write_16(charger, P9221_INT_ENABLE_REG, mask);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);

	return ret;
}

int p9xxx_sw_ramp_icl(struct p9221_charger_data *charger, const int icl_target)
{
	int step, icl_now;

	if (!charger->pdata->has_sw_ramp)
		return 0;

	mutex_lock(&charger->icl_lock);

	icl_now = gvotable_get_current_int_vote(charger->dc_icl_votable);

	dev_dbg(&charger->client->dev, "%s: Set ICL %d->%d ==========\n", __func__, icl_now, icl_target);

	step = (icl_target > icl_now) ? P9XXX_SW_RAMP_ICL_STEP_UA : -P9XXX_SW_RAMP_ICL_STEP_UA;

	while (icl_now != icl_target && charger->online) {
		dev_dbg(&charger->client->dev, "%s: step=%d, get_current_vote=%d\n",
			 __func__,step, gvotable_get_current_int_vote(charger->dc_icl_votable));

		if (abs(icl_target - icl_now) <= P9XXX_SW_RAMP_ICL_STEP_UA)
			icl_now = icl_target;
		else
			icl_now += step;

		dev_dbg(&charger->client->dev, "%s: Voting ICL %duA (t=%d)\n", __func__, icl_now, icl_target);

		gvotable_cast_int_vote(charger->dc_icl_votable, P9221_RAMP_VOTER, icl_now, true);
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
	}

	mutex_unlock(&charger->icl_lock);

	if (!charger->online)
		return -ENODEV;

	dev_dbg(&charger->client->dev, "%s: P9221_RAMP_VOTER=%d, get_current_int_vote=%d ==========\n",
                 __func__,
		 gvotable_get_int_vote(charger->dc_icl_votable, P9221_RAMP_VOTER),
		 gvotable_get_current_int_vote(charger->dc_icl_votable));

	return 0;
}

static int p9221_set_dc_icl(struct p9221_charger_data *charger)
{
	int icl, ret;

	if (!charger->dc_icl_votable) {
		charger->dc_icl_votable =
			gvotable_election_get_handle("DC_ICL");
		if (!charger->dc_icl_votable) {
			dev_err(&charger->client->dev,
				"Could not get votable: DC_ICL\n");
			return -ENODEV;
		}
	}

	/* Default to BPP ICL */
	icl = charger->chip_set_bpp_icl(charger);

	if (charger->chg_on_rtx)
		icl = P9221_DC_ICL_RTX_UA;

	if (charger->icl_ramp)
		icl = charger->icl_ramp_ua;

	/* forced limit */
	if (charger->dc_icl_bpp)
		icl = charger->dc_icl_bpp;

	if (p9221_is_epp(charger))
		icl = charger->dc_icl_epp_neg;

	if (p9221_is_epp(charger) && charger->dc_icl_epp)
		icl = charger->dc_icl_epp;

 	if (charger->icl_ramp && charger->icl_ramp_alt_ua)
		icl = charger->icl_ramp_alt_ua;

	dev_info(&charger->client->dev, "Voting ICL %duA ramp=%d, alt_ramp=%d\n",
		icl, charger->icl_ramp, charger->icl_ramp_alt_ua);

	if (charger->pdata->has_sw_ramp && !charger->icl_ramp) {
		dev_dbg(&charger->client->dev, "%s: voter=%s\n", __func__, P9221_WLC_VOTER);
		ret = p9xxx_sw_ramp_icl(charger, icl);
		if (ret < 0)
			gvotable_cast_int_vote(charger->dc_icl_votable,
					       P9221_RAMP_VOTER, 0, false);
	}

	if (charger->icl_ramp)
		gvotable_cast_int_vote(charger->dc_icl_votable,
				       DCIN_AICL_VOTER, icl, true);

	ret = gvotable_cast_int_vote(charger->dc_icl_votable,
				     P9221_WLC_VOTER, icl, true);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not vote DC_ICL %d\n", ret);

	if (charger->pdata->has_sw_ramp && !charger->icl_ramp)
		gvotable_cast_int_vote(charger->dc_icl_votable, P9221_RAMP_VOTER, 0, false);

	if (p9221_is_epp(charger)) {
		if (!charger->hda_tz_votable)
			charger->hda_tz_votable = gvotable_election_get_handle(VOTABLE_HDA_TZ);
		if (charger->hda_tz_votable)
			gvotable_cast_int_vote(charger->hda_tz_votable, P9221_WLC_VOTER,
				HDA_TZ_WLC_EPP_3P, true);
	}

	/* Increase the IOUT limit */
	charger->chip_set_rx_ilim(charger, P9221_UA_TO_MA(charger->wlc_ocp));
	logbuffer_log(charger->log, "set current limit to %dUA", charger->wlc_ocp);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not set rx_iout limit reg: %d\n", ret);

	return ret;
}

static enum alarmtimer_restart p9221_auth_dc_icl_alarm_cb(struct alarm *alarm,
							  ktime_t now)
{
	struct p9221_charger_data *charger =
			container_of(alarm, struct p9221_charger_data,
				     auth_dc_icl_alarm);

	/* Alarm is in atomic context, schedule work to complete the task */
	dev_info(&charger->client->dev, "Auth timeout, reset DC_ICL\n");
	schedule_delayed_work(&charger->auth_dc_icl_work, msecs_to_jiffies(100));
	return ALARMTIMER_NORESTART;
}

/*
 * only for Qi 1.3 EPP Tx,
 * < 0 error, 0 no support, != support
 */
static int p9221_check_qi1_3_auth(struct p9221_charger_data *chgr)
{
	u8 temp = 0;
	int ret;

	ret = p9221_reg_read_8(chgr, P9221R5_EPP_TX_CAPABILITY_FLAGS_REG, &temp);
	pr_debug("%s: caps=%x (%d)\n", __func__, temp, ret);
	if (ret < 0)
		return -EIO;

	return !!(temp & P9221R5_EPP_TX_CAPABILITY_FLAGS_AR);
}

static void p9221_auth_dc_icl_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, auth_dc_icl_work.work);
	const ktime_t elap = get_boot_sec() - charger->online_at;
	int auth_check = 0;

	mutex_lock(&charger->auth_lock);

	if (elap >= WLCDC_DEBOUNCE_TIME_S)
		p9221_set_auth_dc_icl(charger, false);

	/* done already */
	if (!charger->auth_delay || !p9221_is_online(charger))
		goto exit_done;

	/*
	 * check_id for Qi 1.2.4 EPP Tx and check Qi1.3 EPP Tx until the id is
	 * available (i.e. return different from -EGAIN)
	 */
	if (p9221_is_epp(charger)) {
		auth_check = p9221_ll_check_id(charger);
		if (auth_check == -EAGAIN)
			auth_check = p9221_check_qi1_3_auth(charger);
	}

	pr_debug("%s: online_at=%lld elap=%lld timeout=%d auth_check=%d\n",
		__func__, charger->online_at, elap, WLCDC_AUTH_CHECK_S,
		auth_check);

	/* b/202213483 retry for WLCDC_AUTH_CHECK_S */
	if (auth_check <= 0 && elap < WLCDC_AUTH_CHECK_S)
		auth_check = -EAGAIN;

	/* p9221_auth_dc_icl_alarm_cb() will reschedule at timeout */
	if (auth_check == 0) {
		p9221_set_auth_dc_icl(charger, false);

		/* TODO: use a different wakesource? */
		pm_relax(charger->dev);
	} else if (auth_check < 0) {
		schedule_delayed_work(&charger->auth_dc_icl_work,
				      msecs_to_jiffies(WLCDC_AUTH_CHECK_INTERVAL_MS));
	} else {
		dev_info(&charger->client->dev,
			 "Auth limit online_at=%lld, will timeout in %llds\n",
			 charger->online_at, WLCDC_DEBOUNCE_TIME_S - elap);
	}

exit_done:
	mutex_unlock(&charger->auth_lock);
}

bool is_ping_freq_fixed_at(struct p9221_charger_data *charger, u32 khz)
{
	int ret;
	u32 val;

	ret = charger->chip_get_ping_freq(charger, &val);

	if (ret < 0)
		return false;

	if ((val > khz + 2) || (val < khz - 2))
		return false;

	dev_info(&charger->client->dev, "Fixed ping freq = %dkhz\n", val);

	return true;
}

static enum alarmtimer_restart p9221_icl_ramp_alarm_cb(struct alarm *alarm,
						       ktime_t now)
{
	struct p9221_charger_data *charger =
			container_of(alarm, struct p9221_charger_data,
				     icl_ramp_alarm);

	/* should not schedule icl_ramp_work if charge on rtx phone */
	if (charger->chg_on_rtx)
		return ALARMTIMER_NORESTART;

	dev_info(&charger->client->dev, "ICL ramp alarm, ramp=%d\n",
		 charger->icl_ramp);

	/* Alarm is in atomic context, schedule work to complete the task */
	pm_stay_awake(charger->dev);
	schedule_delayed_work(&charger->icl_ramp_work, msecs_to_jiffies(100));

	return ALARMTIMER_NORESTART;
}

static void p9221_icl_ramp_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, icl_ramp_work.work);

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		pm_runtime_put_sync(charger->dev);
		schedule_delayed_work(&charger->icl_ramp_work,
				      msecs_to_jiffies(100));
		dev_dbg(&charger->client->dev, "Ramp reschedule\n");
		return;
	}
	pm_runtime_put_sync(charger->dev);

	dev_info(&charger->client->dev, "ICL ramp work, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = true;
	p9221_set_dc_icl(charger);

	pm_relax(charger->dev);
}

static void p9221_icl_ramp_reset(struct p9221_charger_data *charger)
{
	dev_info(&charger->client->dev, "ICL ramp reset, ramp=%d\n",
		 charger->icl_ramp);

	charger->icl_ramp = false;

	if (alarm_try_to_cancel(&charger->icl_ramp_alarm) < 0)
		dev_warn(&charger->client->dev, "Couldn't cancel icl_ramp_alarm\n");
	cancel_delayed_work(&charger->icl_ramp_work);
}

static void p9221_icl_ramp_start(struct p9221_charger_data *charger)
{
	const bool no_ramp = charger->pdata->icl_ramp_delay_ms == -1 ||
			     !charger->icl_ramp_ua;

	/* Only ramp on BPP at this time */
	if (p9221_is_epp(charger) || no_ramp)
		return;

	if (charger->pdata->freq_108_disable_ramp && is_ping_freq_fixed_at(charger, 108))
		return;

	if (charger->pdata->magsafe_optimized && is_ping_freq_fixed_at(charger, 128))
		return;

	p9221_icl_ramp_reset(charger);

	dev_info(&charger->client->dev, "ICL ramp set alarm %dms, %dua, ramp=%d\n",
		 charger->pdata->icl_ramp_delay_ms, charger->icl_ramp_ua,
		 charger->icl_ramp);

	alarm_start_relative(&charger->icl_ramp_alarm,
			     ms_to_ktime(charger->pdata->icl_ramp_delay_ms));
}

static void p9221_set_online(struct p9221_charger_data *charger)
{
	int ret;
	u8 cid = 5;

	dev_info(&charger->client->dev, "Set online\n");

	mutex_lock(&charger->stats_lock);
	charger->online = true;
	charger->tx_busy = false;
	charger->tx_done = true;
	charger->rx_done = false;
	charger->cc_data_lock.cc_use = false;
	charger->cc_data_lock.cc_rcv_at = 0;
	charger->last_capacity = -1;
	charger->online_at = get_boot_sec();
	charger->dcin_waitcnt = P9221_DCIN_WAIT_CNT;

	/* reset data for the new charging entry */

	p9221_charge_stats_init(&charger->chg_data);
	mutex_unlock(&charger->stats_lock);

	if (charger->pdata->hda_tz_wlc) {
		if (!charger->hda_tz_votable)
			charger->hda_tz_votable = gvotable_election_get_handle(VOTABLE_HDA_TZ);
		gvotable_cast_int_vote(charger->hda_tz_votable, P9221_WLC_VOTER,
				HDA_TZ_WLC_ADAPTER, true);
	}

	ret = p9221_reg_read_8(charger, P9221_CUSTOMER_ID_REG, &cid);
	if (ret)
		dev_err(&charger->client->dev, "Could not get ID: %d\n", ret);
	else
		charger->cust_id = cid;

	dev_info(&charger->client->dev, "P9221 cid: %02x\n", charger->cust_id);

	ret = p9221_enable_interrupts(charger);
	if (ret)
		dev_err(&charger->client->dev,
			"Could not enable interrupts: %d\n", ret);

	/* NOTE: depends on _is_epp() which is not valid until DC_IN */
	p9221_write_fod(charger);

	cancel_delayed_work(&charger->dcin_pon_work);

	charger->alignment_capable = ALIGN_MFG_CHECKING;
	charger->align = WLC_ALIGN_CENTERED;
	charger->alignment = -1;
	logbuffer_log(charger->log, "align: state: %s",
		      align_status_str[charger->align]);
	schedule_work(&charger->uevent_work);

	schedule_delayed_work(&charger->charge_stats_hda_work,
			      msecs_to_jiffies(P9221_CHARGE_STATS_TIMEOUT_MS));

	p9221_wcin_inlim(charger);
}

static int p9221_set_bpp_vout(struct p9221_charger_data *charger)
{
	u32 vout_mv;
	int ret, loops;
	const u32 vout_5000mv = 5000;

	for (loops = 0; loops < 10; loops++) {
		ret = charger->chip_set_vout_max(charger, vout_5000mv);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot set VOUT (%d)\n", ret);
			return ret;
		}

		ret = charger->chip_get_vout_max(charger, &vout_mv);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot read VOUT (%d)\n", ret);
			return ret;
		}

		if (vout_mv == vout_5000mv)
			return 0;

		msleep(10);
	}

	return -ETIMEDOUT;
}

/* return <0 on error, 0 on done, 1 on keep trying */
static int p9221_notifier_check_neg_power(struct p9221_charger_data *charger)
{
	u8 np8;
	int ret;
	u16 status_reg;

	ret = p9221_reg_read_8(charger, P9221R5_EPP_CUR_NEGOTIATED_POWER_REG,
			       &np8);
	if (ret < 0) {
		dev_err(&charger->client->dev,
			"cannot read EPP_NEG_POWER (%d)\n", ret);
		return -EIO;
	}

	if (np8 >= P9221_NEG_POWER_10W) {
		u16 mfg;

		ret = p9xxx_chip_get_tx_mfg_code(charger, &mfg);
		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot read MFG_CODE (%d)\n", ret);
			return -EIO;
		}


		/* EPP unless dealing with P9221_PTMC_EPP_TX_1912 */
		charger->force_bpp = (mfg == P9221_PTMC_EPP_TX_1912);
		dev_info(&charger->client->dev, "np=%x mfg=%x fb=%d\n",
			 np8, mfg, charger->force_bpp);
		goto done;
	}

	ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
	if (ret) {
		dev_err(&charger->client->dev,
			"failed to read P9221_STATUS_REG reg: %d\n",
			ret);
		return ret;
	}

	/* VOUT for standard BPP comes much earlier that VOUT for EPP */
	if (!(status_reg & charger->ints.vout_changed_bit))
		return 1;

	/* normal BPP TX or EPP at less than 10W */
	charger->force_bpp = true;
	dev_info(&charger->client->dev,
			"np=%x normal BPP or EPP less than 10W (%d)\n",
			np8, ret);

done:
	if (charger->force_bpp) {
		ret = p9221_set_bpp_vout(charger);
		if (ret)
			dev_err(&charger->client->dev,
				"cannot change VOUT (%d)\n", ret);
	}

	return 0;
}

static int p9xxx_set_vout_iop(struct p9221_charger_data *charger)
{
	int ret = 0;
	u32 vout_mv;

	ret = charger->chip_get_vout_max(charger, &vout_mv);
	if (ret < 0)
		goto exit;

	if (charger->pdata->set_iop_vout_bpp > 0 && vout_mv == P9XXX_VOUT_5000MV)
		ret = charger->chip_set_vout_max(charger, charger->pdata->set_iop_vout_bpp);
	else if (charger->pdata->set_iop_vout_epp > 0 && vout_mv == P9XXX_VOUT_10000MV)
		ret = charger->chip_set_vout_max(charger, charger->pdata->set_iop_vout_epp);

	if (charger->pdata->freq_109_vout > 0 && is_ping_freq_fixed_at(charger, 109) &&
	    !p9221_is_epp(charger))
		ret = charger->chip_set_vout_max(charger, charger->pdata->freq_109_vout);
exit:
	if (ret < 0)
		dev_dbg(&charger->client->dev, "Fail to change VOUT\n");

	return ret;
}

/* 2 P9221_NOTIFIER_DELAY_MS from VRECTON */
static void p9221_notifier_check_dc(struct p9221_charger_data *charger)
{
	int ret, dc_in;

	if ((charger->chip_id == P9221_CHIP_ID) && charger->check_np) {

		ret = p9221_notifier_check_neg_power(charger);
		if (ret > 0) {
			ret = schedule_delayed_work(&charger->notifier_work,
				msecs_to_jiffies(P9221_CHECK_NP_DELAY_MS));
			if (ret == 0)
				return;

			dev_err(&charger->client->dev,
				"cannot reschedule check_np (%d)\n", ret);
		}

		/* done */
		charger->check_np = false;
	}

	dc_in = p9221_has_dc_in(charger);
	if (dc_in < 0) {
          	dev_info(&charger->client->dev, "reschedule it(%d)\n", dc_in);
		schedule_delayed_work(&charger->notifier_work,
				    msecs_to_jiffies(P9221_DCIN_RETRY_DELAY_MS));
		return;
	}

	dev_info(&charger->client->dev, "dc status is %d\n", dc_in);
	charger->check_dc = false;
	/*
	 * We now have confirmation from DC_IN, kill the timer, charger->online
	 * will be set by this function.
	 */
	cancel_delayed_work(&charger->dcin_work);
	del_timer(&charger->vrect_timer);

	mutex_lock(&charger->irq_det_lock);
	if (charger->online_spoof && dc_in == 1) {
		/* stop spoofing is queued */
		if (!cancel_delayed_work(&charger->stop_online_spoof_work))
			dev_err(&charger->client->dev,
				"Error: no spoof work even though spoof=1 && dc=1\n");
		logbuffer_prlog(charger->log, "dc=1: online_spoof=0");
		disable_irq_wake(charger->pdata->irq_det_int);
		disable_irq(charger->pdata->irq_det_int);
		charger->online_spoof = false;
		gvotable_cast_bool_vote(charger->wlc_spoof_votable, "WLC", false);
	}
	mutex_unlock(&charger->irq_det_lock);

	if (charger->log) {
		u32 vout_uv;
		u32 vout_mv;

		ret = charger->chip_get_vout(charger, &vout_mv);
		if (!ret)
			vout_uv = P9221_MV_TO_UV(vout_mv);

		logbuffer_log(charger->log,
			      "check_dc: online=%d present=%d VOUT=%uuV (%d)",
			      charger->online, dc_in,
			      (ret == 0) ? vout_uv : 0, ret);
	}

	/*
	 * Always write FOD, check dc_icl, send CSP
	 */
	if (dc_in) {
		if (p9221_is_epp(charger)) {
			charger->chip_check_neg_power(charger);
			/*
			 * When WLC online && EPP, adjust minimum on time(MOT) to 40%
			 * for mitigate LC node voltage overshoot
			 */
			ret = p9xxx_chip_set_mot_reg(charger, P9412_MOT_40PCT);
			if (ret < 0 && ret != -ENOTSUPP)
				dev_warn(&charger->client->dev,
					 "Fail to set MOT register(%d)\n", ret);
		}

		if (charger->pdata->has_sw_ramp && !charger->sw_ramp_done) {
			gvotable_cast_int_vote(charger->dc_icl_votable,
					       P9221_RAMP_VOTER,
					       P9XXX_SW_RAMP_ICL_START_UA, true);
			charger->sw_ramp_done = true;
		}
		charger->chip_magsafe_optimized(charger);
		p9221_set_dc_icl(charger);
		p9xxx_set_vout_iop(charger);
		if (!p9221_is_epp(charger))
			p9xxx_chip_set_ask_mod_fet(charger, charger->pdata->ask_mod_fet);
		charger->fod_mode = -1;
		p9221_write_fod(charger);
		if (!charger->dc_icl_bpp)
			p9221_icl_ramp_start(charger);
		charger->chip_set_cmfet(charger);

		ret = p9221_reg_read_16(charger, P9221_OTP_FW_MINOR_REV_REG,
					&charger->fw_rev);
		if (ret)
			dev_err(&charger->client->dev,
				"Could not get FW_REV: %d\n", ret);
	}

	/* We may have already gone online during check_det */
	if (charger->online == dc_in)
		goto out;

	if (dc_in)
		p9221_set_online(charger);
	else
		p9221_set_offline(charger);

out:
	dev_info(&charger->client->dev, "trigger wc changed on:%d in:%d\n",
		 charger->online, dc_in);
	power_supply_changed(charger->wc_psy);

	p9221_wcin_inlim(charger);
}

/* P9221_NOTIFIER_DELAY_MS from VRECTON */
static bool p9221_notifier_check_det(struct p9221_charger_data *charger)
{
	bool relax = true;

	del_timer(&charger->vrect_timer);

	if (charger->online && !charger->ben_state)
		goto done;

	dev_info(&charger->client->dev, "detected wlc, trigger wc changed\n");

	/* b/130637382 workaround for 2622,2225,2574,1912 */
	charger->check_np = true;
	/* will send out a FOD but is_epp() is still invalid */
	p9221_set_online(charger);
	power_supply_changed(charger->wc_psy);

	/* Check dc-in every seconds as long as we are in field. */
	dev_info(&charger->client->dev, "start dc-in timer\n");
	cancel_delayed_work_sync(&charger->dcin_work);
	schedule_delayed_work(&charger->dcin_work,
			      msecs_to_jiffies(P9221_DCIN_TIMEOUT_MS));
	relax = false;

done:
	charger->check_det = false;

	return relax;
}

/* Resonance Frequency: This new value needed for Qi 1.3 */
static void p9xxx_write_resonance_freq(struct p9221_charger_data *charger)
{
	const int rf_value = charger->pdata->rf_value;
	int ret;

	if (rf_value == -1)
		return;

	ret = p9xxx_chip_set_resonance_freq_reg(charger, rf_value);
	if (ret < 0)
		dev_err(&charger->client->dev, "cannot write resonance_freq=%d (%d)\n",
			rf_value, ret);
}

static void p9xxx_write_q_factor(struct p9221_charger_data *charger)
{
	int ret, q_factor;

	if (charger->pdata->q_value == -1 && charger->de_q_value == 0)
		return;

	q_factor = (charger->de_q_value > 0) ?
		   charger->de_q_value : charger->pdata->q_value;

	ret = p9xxx_chip_set_q_factor_reg(charger, q_factor);
	if (ret < 0)
		dev_err(&charger->client->dev,
			"cannot write Q=%d (%d)\n",
			q_factor, ret);
}

static void p9xxx_update_q_factor(struct p9221_charger_data *charger)
{
	int ret, i, q = 0;

	if (charger->pdata->tx_4191q <= 0 &&
	    charger->pdata->tx_1801q <= 0 &&
	    charger->pdata->tx_2356q <= 0)
		return;

	for (i = 0; i < 5; i += 1) {
		ret = p9xxx_chip_get_tx_mfg_code(charger, &charger->mfg);
		if (ret == 0 && charger->mfg != 0)
			break;
		usleep_range(100 * USEC_PER_MSEC, 120 * USEC_PER_MSEC);
	}

	switch (charger->mfg) {
	case P9221_PTMC_EPP_TX_4191:
		q = charger->pdata->tx_4191q;
		break;
	case P9221_PTMC_EPP_TX_1801:
		q = charger->pdata->tx_1801q;
		break;
	case P9221_PTMC_EPP_TX_2356:
		q = charger->pdata->tx_2356q;
		break;
	default:
		return;
	}

	if (q <= 0)
		return;

	ret = p9xxx_chip_set_q_factor_reg(charger, q);
	dev_info(&charger->client->dev, "update Q factor=%d(mfg=%x) ret=%d\n",
		 q, charger->mfg, ret);
}

/* pm_stay_awake(charger->dev) is needed for schedule notifier_work */
static void p9221_notifier_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, notifier_work.work);
	bool relax = true;
	int ret;

	dev_info(&charger->client->dev, "Notifier work: on:%d ben:%d dc:%d np:%d det:%d\n",
		 charger->online,
		 charger->ben_state,
		 charger->check_dc, charger->check_np,
		 charger->check_det);

	if (!charger->votable_init_done) {
		charger->votable_init_done = p9xxx_find_votable(charger);
		if (charger->votable_init_done)
			dev_dbg(&charger->client->dev, "p9xxx_find_votable is done\n");
	}

	charger->send_eop = gvotable_get_int_vote(charger->dc_icl_votable,
						  THERMAL_DAEMON_VOTER) == 0;
	if (charger->send_eop && !charger->online) {
		dev_info(&charger->client->dev, "WLC should be disabled!\n");
		p9221_wlc_disable(charger, 1, P9221_EOP_UNKNOWN);
		goto done_relax;
	}

	/* Calibrate light load */
	if (charger->pdata->light_load)
		p9xxx_chip_set_light_load_reg(charger, P9222_LIGHT_LOAD_VALUE);

	if (charger->pdata->rf_value != -1)
		p9xxx_write_resonance_freq(charger);
	p9xxx_write_q_factor(charger);
	if (charger->pdata->tx_4191q > 0)
		p9xxx_update_q_factor(charger);

	if (charger->pdata->epp_rp_value != -1) {

		ret = charger->chip_renegotiate_pwr(charger);
		if (ret < 0)
			dev_err(&charger->client->dev,
				"cannot renegotiate power=%d (%d)\n",
				 charger->pdata->epp_rp_value, ret);
	}

	if (charger->log) {
		u32 vrect_mv, tx_pwr;
		int rc;

		ret = charger->chip_get_vrect(charger, &vrect_mv);
		rc = charger->chip_get_tx_epp_guarpwr(charger, &tx_pwr);
		logbuffer_log(charger->log,
			      "notifier: on:%d ben:%d dc:%d det:%d VRECT=%uuV TxGuarPwr=%uW (%d)",
			      charger->online,
			      charger->ben_state,
			      charger->check_dc, charger->check_det,
			      (ret == 0) ? P9221_MV_TO_UV(vrect_mv) : 0,
			      (rc == 0) ? tx_pwr / 1000 : 0, ret);
		}

	if (charger->check_det)
		relax = p9221_notifier_check_det(charger);

	if (charger->check_dc)
		p9221_notifier_check_dc(charger);

done_relax:
	if (relax)
		pm_relax(charger->dev);
}

static size_t p9221_add_buffer(char *buf, u32 val, size_t count, int ret,
			       const char *name, char *fmt)
{
	int added = 0;

	added += scnprintf(buf + count, PAGE_SIZE - count, "%s", name);
	count += added;
	if (ret)
		added += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else
		added += scnprintf(buf + count, PAGE_SIZE - count, fmt, val);

	return added;
}

static ssize_t p9221_add_reg_buffer(struct p9221_charger_data *charger,
				    char *buf, size_t count, u16 reg, int width,
				    bool cooked, const char *name, char *fmt)
{
	u32 val;
	int ret;

	if (width == 16) {
		u16 val16 = 0;

		ret = p9221_reg_read_16(charger, reg, &val16);
		val = val16;
	} else {
		u8 val8 = 0;

		ret = p9221_reg_read_8(charger, reg, &val8);
		val = val8;
	}

	return p9221_add_buffer(buf, val, count, ret, name, fmt);
}

static ssize_t p9221_show_version(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0;
	int i;
	u8 val8 = 0;

	if (!p9221_is_online(charger))
		return -ENODEV;

	count += p9221_add_reg_buffer(charger, buf, count, P9221_CHIP_ID_REG,
				      16, 0, "chip id    : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CHIP_REVISION_REG, 8, 0,
				      "chip rev   : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_CUSTOMER_ID_REG, 8, 0,
				      "cust id    : ", "%02x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MAJOR_REV_REG, 16, 0,
				      "otp fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_OTP_FW_MINOR_REV_REG, 16, 0,
				      "otp fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "otp fw date: ");
	for (i = 0; i < P9221_OTP_FW_DATE_SIZE; i++) {
		p9221_reg_read_8(charger,
				       P9221_OTP_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\notp fw time: ");
	for (i = 0; i < P9221_OTP_FW_TIME_SIZE; i++) {
		p9221_reg_read_8(charger,
				       P9221_OTP_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MAJOR_REV_REG, 16, 0,
				      "\nram fw maj : ", "%04x\n");
	count += p9221_add_reg_buffer(charger, buf, count,
				      P9221_SRAM_FW_MINOR_REV_REG, 16, 0,
				      "ram fw min : ", "%04x\n");

	count += scnprintf(buf + count, PAGE_SIZE - count, "ram fw date: ");
	for (i = 0; i < P9221_SRAM_FW_DATE_SIZE; i++) {
		p9221_reg_read_8(charger,
				       P9221_SRAM_FW_DATE_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\nram fw time: ");
	for (i = 0; i < P9221_SRAM_FW_TIME_SIZE; i++) {
		p9221_reg_read_8(charger,
				       P9221_SRAM_FW_TIME_REG + i, &val8);
		if (val8)
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "%c", val8);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(version, 0444, p9221_show_version, NULL);

static ssize_t p9221_show_status(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int count = 0, i;
	int ret;
	u8 tmp[P9221R5_NUM_FOD];
	uint32_t tx_id = 0;
	u32 val32;
	u16 val16;
	u8 val8;

	if (!p9221_is_online(charger))
		return -ENODEV;

	ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &val16);
	count += p9221_add_buffer(buf, val16, count, ret,
				  "status      : ", "%04x\n");

	ret = p9221_reg_read_16(charger, P9221_INT_REG, &val16);
	count += p9221_add_buffer(buf, val16, count, ret,
				  "int         : ", "%04x\n");

	ret = p9221_reg_read_16(charger, P9221_INT_ENABLE_REG, &val16);
	count += p9221_add_buffer(buf, val16, count, ret,
				  "int_enable  : ", "%04x\n");

	ret = charger->chip_get_sys_mode(charger, &val8);
	count += p9221_add_buffer(buf, val8, count, ret,
				  "mode        : ", "%02x\n");

	ret = charger->chip_get_vout(charger, &val32);
	count += p9221_add_buffer(buf, P9221_MV_TO_UV(val32), count, ret,
				  "vout        : ", "%u uV\n");

	ret = charger->chip_get_vrect(charger, &val32);
	count += p9221_add_buffer(buf, P9221_MV_TO_UV(val32), count, ret,
				  "vrect       : ", "%u uV\n");

	ret = charger->chip_get_iout(charger, &val32);
	count += p9221_add_buffer(buf, P9221_MA_TO_UA(val32), count, ret,
				  "iout        : ", "%u uA\n");

	if (charger->ben_state == RTX_BEN_ON)
		ret = charger->chip_get_tx_ilim(charger, &val32);
	else
		ret = charger->chip_get_rx_ilim(charger, &val32);
	count += p9221_add_buffer(buf, P9221_MA_TO_UA(val32), count, ret,
				  "ilim        : ", "%u uA\n");

	ret = charger->chip_get_op_freq(charger, &val32);
	count += p9221_add_buffer(buf, P9221_KHZ_TO_HZ(val32), count, ret,
				  "freq        : ", "%u hz\n");

	if (charger->ben_state == RTX_BEN_ON) {
		ret = charger->chip_get_op_duty(charger, &val32);
		count += p9221_add_buffer(buf, val32, count, ret,
					  "duty        : ", "%u %%\n");
		ret = charger->chip_get_op_bridge(charger, &val8);
		count += p9221_add_buffer(buf, val8, count, ret,
					  "HB/FB(0/1)  : ", "%u\n");
		ret = charger->chip_get_tx_pwr(charger, &val16);
		count += p9221_add_buffer(buf, val16, count, ret,
					  "curr_tx_pwr : ", "%u mW\n");
		ret = charger->chip_get_rx_pwr(charger, &val16);
		count += p9221_add_buffer(buf, val16, count, ret,
					  "curr_rx_pwr : ", "%u mW\n");
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_busy     : %d\n", charger->tx_busy);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_done     : %d\n", charger->tx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_done     : %d\n", charger->rx_done);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_len      : %d\n", charger->tx_len);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "rx_len      : %d\n", charger->rx_len);
	p9xxx_chip_get_tx_id(charger, &tx_id);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "tx_id       : %08x (%s)\n", tx_id,
			   p9221_get_tx_id_str(charger));

	ret = charger->chip_get_align_x(charger, &val8);
	count += p9221_add_buffer(buf, val8, count, ret,
				  "align_x     : ", "%u\n");

	ret = charger->chip_get_align_y(charger, &val8);
	count += p9221_add_buffer(buf, val8, count, ret,
				  "align_y     : ", "%u\n");

	/* WLC_DC state */
	if (charger->prop_mode_en) {
		ret = charger->reg_read_8(charger, P9412_PROP_CURR_PWR_REG,
					  &val8);
		count += p9221_add_buffer(buf, val8, count, ret,
					  "curr_pwr_reg: ", "%02x\n");
	}

	/* FOD Register */
	ret = p9xxx_chip_get_fod_reg(charger, tmp, P9221R5_NUM_FOD);
	count += scnprintf(buf + count, PAGE_SIZE - count, "fod         : ");
	if (ret)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "err %d\n", ret);
	else {
		count += p9221_hex_str(tmp, P9221R5_NUM_FOD, buf + count, count,
				       false);
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	/* Device tree FOD entries */
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "dt fod      : (n=%d) ", charger->pdata->fod_num);
	count += p9221_hex_str(charger->pdata->fod, charger->pdata->fod_num,
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\ndt fod-epp  : (n=%d) ",
			   charger->pdata->fod_epp_num);
	count += p9221_hex_str(charger->pdata->fod_epp,
			       charger->pdata->fod_epp_num,
			       buf + count, PAGE_SIZE - count, false);

	for (i = 0; i < charger->pdata->nb_hpp_fod_vol; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "\ndt fod-hpp-%d  : (n=%d) ",
				   i, charger->pdata->hpp_fods[i].num);
		count += p9221_hex_str(charger->pdata->hpp_fods[i].fod,
				       charger->pdata->hpp_fods[i].num,
				       buf + count, PAGE_SIZE - count, false);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "\npp buf      : (v=%d) ", charger->pp_buf_valid);
	count += p9221_hex_str(charger->pp_buf, sizeof(charger->pp_buf),
			       buf + count, PAGE_SIZE - count, false);

	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	return count;
}

static DEVICE_ATTR(status, 0444, p9221_show_status, NULL);

static ssize_t p9221_show_count(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%u\n", charger->count);
}

static ssize_t p9221_store_count(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u8 cnt;

	ret = kstrtou8(buf, 0, &cnt);
	if (ret < 0)
		return ret;
	charger->count = cnt;
	return count;
}

static DEVICE_ATTR(count, 0644, p9221_show_count, p9221_store_count);

static ssize_t p9221_show_icl_ramp_delay_ms(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 charger->pdata->icl_ramp_delay_ms);
}

static ssize_t p9221_store_icl_ramp_delay_ms(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ms;

	ret = kstrtou32(buf, 10, &ms);
	if (ret < 0)
		return ret;
	charger->pdata->icl_ramp_delay_ms = ms;
	return count;
}

static DEVICE_ATTR(icl_ramp_delay_ms, 0644,
		   p9221_show_icl_ramp_delay_ms,
		   p9221_store_icl_ramp_delay_ms);

static ssize_t p9221_show_icl_ramp_ua(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->icl_ramp_ua);
}

static ssize_t p9221_store_icl_ramp_ua(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;
	charger->icl_ramp_ua = ua;
	return count;
}

static DEVICE_ATTR(icl_ramp_ua, 0644,
		   p9221_show_icl_ramp_ua, p9221_store_icl_ramp_ua);

static ssize_t p9221_show_addr(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", charger->addr);
}

static ssize_t p9221_store_addr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 addr;

	ret = kstrtou16(buf, 16, &addr);
	if (ret < 0)
		return ret;
	charger->addr = addr;
	return count;
}

static DEVICE_ATTR(addr, 0644, p9221_show_addr, p9221_store_addr);

static ssize_t p9221_show_data(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int ret;
	int i;
	ssize_t len = 0;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	ret = p9221_reg_read_n(charger, charger->addr, reg, charger->count);
	if (ret)
		return ret;

	for (i = 0; i < charger->count; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02x: %02x\n",
				 charger->addr + i, reg[i]);
	}
	return len;
}

static ssize_t p9221_store_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u8 reg[256];
	int i = 0;
	int ret = 0;
	char *data;
	char *tmp_buf;

	if (!charger->count || (charger->addr > (0xFFFF - charger->count)))
		return -EINVAL;

	if (!p9221_is_online(charger))
		return -ENODEV;

	tmp_buf = kstrdup(buf, GFP_KERNEL);
	data = tmp_buf;
	if (!data)
		return -ENOMEM;

	while (data && i < charger->count) {
		char *d = strsep(&data, " ");

		if (*d) {
			ret = kstrtou8(d, 16, &reg[i]);
			if (ret)
				break;
			i++;
		}
	}
	if ((i != charger->count) || ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = p9221_reg_write_n(charger, charger->addr, reg, charger->count);
	if (ret)
		goto out;
	ret = count;

out:
	kfree(tmp_buf);
	return ret;
}

static DEVICE_ATTR(data, 0644, p9221_show_data, p9221_store_data);

static ssize_t p9221_store_ccreset(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;

	ret = p9221_send_ccreset(charger);
	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR(ccreset, 0200, NULL, p9221_store_ccreset);

static ssize_t p9221_show_rxdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->rx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(rxdone, 0444, p9221_show_rxdone, NULL);

static ssize_t p9221_show_rxlen(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%hu\n", charger->rx_len);
}

static DEVICE_ATTR(rxlen, 0444, p9221_show_rxlen, NULL);

static ssize_t p9221_show_txdone(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_done ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txdone, 0444, p9221_show_txdone, NULL);

static ssize_t p9221_show_txbusy(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->tx_busy ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static DEVICE_ATTR(txbusy, 0444, p9221_show_txbusy, NULL);

static ssize_t p9221_store_txlen(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 len;

	ret = kstrtou16(buf, 16, &len);
	if (ret < 0)
		return ret;

	if (!charger->online)
		return -ENODEV;

	/* Don't send bidi data in cases of BPP and NOT dream-defend */
	if (!p9221_is_epp(charger) && !charger->trigger_power_mitigation)
		return -EINVAL;

	if (!charger->chip_is_calibrated(charger)) {
		dev_warn(&charger->client->dev, "Calibrating for tx\n");
		return -EAGAIN;
	}

	charger->tx_len = len;

	ret = set_renego_state(charger, P9XXX_SEND_DATA);
	if (ret != 0)
		return -EBUSY;
	charger->tx_done = false;
	ret = p9221_send_data(charger);
	if (ret) {
		charger->tx_done = true;
		set_renego_state(charger, P9XXX_AVAILABLE);
		return ret;
	}

	cancel_delayed_work_sync(&charger->tx_work);
	schedule_delayed_work(&charger->tx_work,
			      msecs_to_jiffies(P9221_TX_TIMEOUT_MS));

	return count;
}

static DEVICE_ATTR(txlen, 0200, NULL, p9221_store_txlen);

static ssize_t authtype_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 type;

	ret = kstrtou16(buf, 16, &type);
	if (ret < 0)
		return ret;

	charger->auth_type = type;

	return count;
}

static DEVICE_ATTR_WO(authtype);

static ssize_t p9221_show_force_epp(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	buf[0] = charger->fake_force_epp ? '1' : '0';
	buf[1] = 0;
	return 1;
}

static ssize_t p9221_force_epp(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u16 val;

	ret = kstrtou16(buf, 16, &val);
	if (ret < 0)
		return ret;

	charger->fake_force_epp = (val != 0);

	if (charger->pdata->slct_gpio >= 0)
		gpio_set_value_cansleep(charger->pdata->slct_gpio,
			       charger->fake_force_epp ? 1 : 0);
	return count;
}

static DEVICE_ATTR(force_epp, 0600, p9221_show_force_epp, p9221_force_epp);

static ssize_t dc_icl_epp_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->dc_icl_epp);
}

static ssize_t dc_icl_epp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;

	charger->dc_icl_epp = ua;

	if (charger->dc_icl_votable && p9221_is_epp(charger))
		gvotable_cast_int_vote(charger->dc_icl_votable, P9221_WLC_VOTER,
				       charger->dc_icl_epp, true);

	return count;
}

static DEVICE_ATTR_RW(dc_icl_epp);

static ssize_t p9221_show_dc_icl_bpp(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->dc_icl_bpp);
}

static ssize_t p9221_set_dc_icl_bpp(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;

	charger->dc_icl_bpp = ua;

	if (charger->dc_icl_votable && !p9221_is_epp(charger))
		gvotable_cast_int_vote(charger->dc_icl_votable, P9221_WLC_VOTER,
				       charger->dc_icl_bpp, true);

	return count;
}

static DEVICE_ATTR(dc_icl_bpp, 0644,
		   p9221_show_dc_icl_bpp, p9221_set_dc_icl_bpp);

static ssize_t p9221_show_alignment(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->alignment == -1)
		p9221_init_align(charger);

	if ((charger->align != WLC_ALIGN_CENTERED) ||
	    (charger->alignment == -1))
		return scnprintf(buf, PAGE_SIZE, "%s\n",
				 align_status_str[charger->align]);
	else
		return scnprintf(buf, PAGE_SIZE, "%d\n", charger->alignment);
}

static DEVICE_ATTR(alignment, 0444, p9221_show_alignment, NULL);

static ssize_t operating_freq_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0, val = 0;

	ret = p9221_ready_to_read(charger);
	if (!ret) {
		ret = charger->chip_get_op_freq(charger, &val);
		if (!ret)
			val = P9221_KHZ_TO_HZ(val);
	}

	if (ret)
		val = ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static DEVICE_ATTR_RO(operating_freq);

static ssize_t aicl_delay_ms_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->aicl_delay_ms);
}

static ssize_t aicl_delay_ms_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	u32 t;

	ret = kstrtou32(buf, 10, &t);
	if (ret < 0)
		return ret;

	charger->aicl_delay_ms = t;

	return count;
}

static DEVICE_ATTR_RW(aicl_delay_ms);

static ssize_t aicl_icl_ua_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->aicl_icl_ua);
}

static ssize_t aicl_icl_ua_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	u32 ua;

	ret = kstrtou32(buf, 10, &ua);
	if (ret < 0)
		return ret;

	charger->aicl_icl_ua = ua;

	return count;
}

static DEVICE_ATTR_RW(aicl_icl_ua);

static ssize_t ptmc_id_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	p9382_get_ptmc_id_str(charger->ptmc_id_str, PAGE_SIZE, charger);
	return scnprintf(buf, PAGE_SIZE, "%s\n", charger->ptmc_id_str);
}

static DEVICE_ATTR_RO(ptmc_id);

static ssize_t features_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	u64 id, ft;
	int ret;

	ret = sscanf(buf, "%llx:%llx", &id, &ft);
	if (ret != 2)
		return -EINVAL;

	pr_debug("%s: tx_id=%llx, ft=%llx", __func__, id, ft);

	if (id) {
		mutex_lock(&chg_fts->feat_lock);
		if (id != 1)
			feature_update_cache(&charger->chg_features, id, ft);
		/*
		 * disable prefill of feature cache on first use of the API,
		 * TODO: possibly clear the cache as well.
		 */
		if (charger->pdata->feat_compat_mode) {
			dev_info(charger->dev, "compat mode off\n");
			charger->pdata->feat_compat_mode = false;
		}
		mutex_unlock(&chg_fts->feat_lock);
	} else {
		mutex_lock(&charger->auth_lock);
		/* remove auth_icl after AUTH is done */
		p9221_set_auth_dc_icl(charger, false);
		mutex_unlock(&charger->auth_lock);
		ret = feature_update_session(charger, ft);
		if (ret < 0)
			count = ret;
	}

	return count;
}

static ssize_t features_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	struct p9221_charger_feature *chg_fts = &charger->chg_features;
	struct p9221_charger_feature_entry *entry = &chg_fts->entries[0];
	u64 session_features = chg_fts->session_features;
	int idx;
	ssize_t len = 0;

	if (!charger->wlc_dc_enabled)
		session_features &= ~WLCF_FAST_CHARGE;

	len += scnprintf(buf + len, PAGE_SIZE - len, "0:%llx\n",
			 session_features);

	for (idx = 0; idx < chg_fts->num_entries; idx++, entry++) {
		if (entry->quickid == 0 || entry->features == 0)
			continue;
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%llx:%llx\n",
				 entry->quickid,
				 entry->features);
	}
	return len;
}

static DEVICE_ATTR_RW(features);

static ssize_t mitigate_threshold_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%u\n", charger->mitigate_threshold);
}

static ssize_t mitigate_threshold_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	u8 threshold;

	ret = kstrtou8(buf, 0, &threshold);
	if (ret < 0)
		return ret;
	charger->mitigate_threshold = threshold;
	return count;
}

static DEVICE_ATTR_RW(mitigate_threshold);

/*
 * b/205193265 AP needs to wait for Cal-mode1 and mode2 to clear (calibration
 * to be complete) before attempting both authentication and re-negotiation.
 * The HAL API calls for authentication can check this node.
 */
static ssize_t wpc_ready_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	/* Skip it on BPP */
	if (!p9221_is_epp(charger) ||
	    !p9221_check_wpc_rev13(charger) ||
	    (!charger->pdata->has_wlc_dc && !charger->pdata->gpp_enhanced))
		return scnprintf(buf, PAGE_SIZE, "N\n");

	return scnprintf(buf, PAGE_SIZE, "%c\n",
			 charger->chip_is_calibrated(charger) ? 'Y' : 'N');
}

static DEVICE_ATTR_RO(wpc_ready);

/* ------------------------------------------------------------------------ */
static ssize_t rx_lvl_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (!charger->pdata->has_rtx)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->rtx_csp);
}

static DEVICE_ATTR_RO(rx_lvl);

/* requires mutex_unlock(&charger->rtx_lock); */
static enum p9382_rtx_state p9xxx_get_rtx_status(struct p9221_charger_data *charger)
{
	int ext_bst_on = 0, rtx_err = charger->rtx_err;

	if (rtx_err & RTX_OVER_TEMP_BIT)
		return RTX_DISABLED;

	if (!charger->pdata->has_rtx || rtx_err & RTX_CHRG_NOTSUP_BIT)
		return RTX_NOTSUPPORTED;

	/* external boost is on but not for rtx */
	if (charger->pdata->ben_gpio > 0)
		ext_bst_on = gpio_get_value_cansleep(charger->pdata->ben_gpio);

	if (p9221_is_online(charger))
		return (charger->ben_state || charger->rtx_reset_cnt) ? RTX_ACTIVE : RTX_DISABLED;

	if (!ext_bst_on)
		return RTX_AVAILABLE;

	return RTX_DISABLED;
}

static ssize_t rtx_status_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	static const char * const rtx_state_text[] = {
		"not support", "available", "active", "disabled" };
	enum p9382_rtx_state rtx_status;

	mutex_lock(&charger->rtx_lock);
	rtx_status = p9xxx_get_rtx_status(charger);
	mutex_unlock(&charger->rtx_lock);

	if (p9xxx_rtx_gpio_is_state(charger, RTX_RETRY))
		rtx_status = RTX_ACTIVE;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 rtx_state_text[rtx_status]);
}

static DEVICE_ATTR_RO(rtx_status);

static ssize_t is_rtx_connected_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	u16 status_reg = 0;
	bool attached = 0;

	if (!charger->pdata->has_rtx)
		return -ENODEV;

	if (charger->ben_state)
		p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);

	attached = status_reg & charger->ints.rx_connected_bit;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 attached ? "connected" : "disconnect");
}

static DEVICE_ATTR_RO(is_rtx_connected);

static ssize_t rtx_err_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int err = RTX_NO_ERROR, rtx_err = charger->rtx_err;

	if (rtx_err & RTX_BATT_LOW_BIT)
		err = RTX_BATT_LOW;
	else if (rtx_err & RTX_OVER_TEMP_BIT)
		err = RTX_OVER_TEMP;
	else if (rtx_err & RTX_TX_CONFLICT_BIT)
		err = RTX_TX_CONFLICT;
	else if (rtx_err & RTX_HARD_OCP_BIT)
		err = RTX_HARD_OCP;
	else if (rtx_err & RTX_CHRG_NOTSUP_BIT)
		err = RTX_CHRG_NOT_SUP;

	return scnprintf(buf, PAGE_SIZE, "%d\n", err);
}

static DEVICE_ATTR_RO(rtx_err);

static ssize_t ldo_en_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->ldo_en_gpio < 0)
		return -ENODEV;

	value = gpio_get_value_cansleep(charger->pdata->ldo_en_gpio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t ldo_en_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->pdata->ldo_en_gpio < 0)
		return -ENODEV;

	gpio_set_value_cansleep(charger->pdata->ldo_en_gpio, buf[0] != '0');

	return count;
}
static DEVICE_ATTR_RW(ldo_en);

static ssize_t qien_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->qien_gpio < 0)
		return -ENODEV;

	value = gpio_get_value_cansleep(charger->pdata->qien_gpio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t qien_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->pdata->qien_gpio < 0)
		return -ENODEV;

	gvotable_cast_bool_vote(charger->wlc_disable_votable, "DEBUG_VOTER", buf[0] != '0');

	return count;
}
static DEVICE_ATTR_RW(qien);

static ssize_t qi_vbus_en_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->qi_vbus_en < 0)
		return -ENODEV;

	value = gpio_get_value_cansleep(charger->pdata->qi_vbus_en);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t qi_vbus_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->qi_vbus_en < 0)
		return -ENODEV;

	value = (buf[0] != '0') ^ charger->pdata->qi_vbus_en_act_low;

	gpio_set_value_cansleep(charger->pdata->qi_vbus_en, value);

	return count;
}
static DEVICE_ATTR_RW(qi_vbus_en);

static ssize_t ext_ben_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->ext_ben_gpio < 0)
		return -ENODEV;

	value = gpio_get_value_cansleep(charger->pdata->ext_ben_gpio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t ext_ben_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->pdata->ext_ben_gpio < 0)
		return -ENODEV;

	gpio_set_value_cansleep(charger->pdata->ext_ben_gpio, buf[0] != '0');

	return count;
}
static DEVICE_ATTR_RW(ext_ben);

static ssize_t rtx_sw_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int value;

	if (charger->pdata->switch_gpio < 0)
		return -ENODEV;

	value = gpio_get_value_cansleep(charger->pdata->switch_gpio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", value != 0);
}

static ssize_t rtx_sw_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (charger->pdata->switch_gpio < 0)
		return -ENODEV;

	/* TODO: better test on rX mode */
	if (charger->online) {
		dev_err(&charger->client->dev, "invalid rX state");
		return -EINVAL;
	}

	gpio_set_value_cansleep(charger->pdata->switch_gpio, buf[0] != '0');

	return count;
}

static DEVICE_ATTR_RW(rtx_sw);

static ssize_t fw_rev_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", charger->fw_rev);
}

static DEVICE_ATTR_RO(fw_rev);

static ssize_t fan_level_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int result = 0;

	if (charger->fan_level_votable)
		result = gvotable_get_current_int_vote(charger->fan_level_votable);

	return scnprintf(buf, PAGE_SIZE, "%d\n", result);
}

static ssize_t fan_level_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret = 0;
	int level;

	ret = kstrtoint(buf, 0, &level);
	if (ret < 0)
		return ret;

	if ((level < FAN_LVL_UNKNOWN) || (level > FAN_LVL_ALARM))
		return -ERANGE;

	if (charger->fan_level_votable) {
		ret = gvotable_cast_int_vote(charger->fan_level_votable, "MSC_USR", level, true);
		if (ret < 0)
			pr_err("MSC_FAN_LVL: fail to set level=%d(ret=%d)\n", level, ret);
	}

	return count;
}

static DEVICE_ATTR_RW(fan_level);

static int irq_det_show(void *data, u64 *val)
{
	struct p9221_charger_data *charger = data;

	*val = charger->det_status;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_irq_det_fops, irq_det_show, NULL, "%lld\n");

static ssize_t p9382_show_rtx_boost(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->ben_state);
}

/* assume that we have 2 GPIO to turn on the boost */
static int p9382_rtx_enable(struct p9221_charger_data *charger, bool enable)
{
	int ret = 0;

	/*
	 * TODO: deprecate the support for rTX on whitefin2 or use a DT entry
	 * to ignore the votable and use ben+switch
	 */

	if (!charger->chg_mode_votable)
		charger->chg_mode_votable =
			gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (charger->chg_mode_votable) {
		ret = gvotable_cast_long_vote(charger->chg_mode_votable,
					      P9221_WLC_VOTER,
					      GBMS_CHGR_MODE_WLC_TX, enable);
		if (charger->pdata->ben_gpio > 0)
			gpio_set_value_cansleep(charger->pdata->ben_gpio, enable);
		return ret;
	}

	if (charger->pdata->ben_gpio > 0)
		gpio_set_value_cansleep(charger->pdata->ben_gpio, enable);
	if (charger->pdata->switch_gpio > 0)
		gpio_set_value_cansleep(charger->pdata->switch_gpio, enable);

	/* some systems provide additional boost_gpio for charging level */
	if (charger->pdata->boost_gpio > 0)
		gpio_set_value_cansleep(charger->pdata->boost_gpio, enable);

	return (charger->pdata->ben_gpio < 0 &&
		charger->pdata->switch_gpio < 0) ? -ENODEV : 0;
}

static int p9382_ben_cfg(struct p9221_charger_data *charger, int cfg)
{
	const int ben_gpio = charger->pdata->ben_gpio;
	const int switch_gpio = charger->pdata->switch_gpio;

	dev_info(&charger->client->dev, "ben_cfg: %d->%d (ben=%d, switch=%d)",
		 charger->ben_state, cfg, ben_gpio, switch_gpio);

	switch (cfg) {
	case RTX_BEN_DISABLED:
		if (charger->ben_state == RTX_BEN_ON)
			p9382_rtx_enable(charger, false);
		else if (ben_gpio == RTX_BEN_ENABLED)
			gpio_set_value_cansleep(ben_gpio, 0);
		charger->ben_state = cfg;
		break;
	case RTX_BEN_ENABLED:
		charger->ben_state = cfg;
		if (ben_gpio >= 0)
			gpio_set_value_cansleep(ben_gpio, 1);
		break;
	case RTX_BEN_ON:
		charger->ben_state = cfg;
		p9382_rtx_enable(charger, true);
		break;
	case FACTORY_BOOST_ENABLED:
		charger->ben_state = cfg;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t p9382_set_rtx_boost(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	const int state = buf[0] - '0';
	int ret;

	/* always ok to disable */
	if (state && charger->online && !charger->ben_state) {
		dev_err(&charger->client->dev, "invalid rX state");
		return -ENODEV;
	}

	/* 0 -> BEN_DISABLED, 1 -> BEN_ON */
	ret = p9382_ben_cfg(charger, state);
	if (ret < 0)
		count = ret;

	return count;
}

static DEVICE_ATTR(rtx_boost, 0644, p9382_show_rtx_boost, p9382_set_rtx_boost);

static ssize_t p9221_show_chg_stats(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int i, len = -ENODATA;

	mutex_lock(&charger->stats_lock);

	if (charger->chg_data.cur_soc < 0)
		goto enodata_done;

	len = p9221_chg_data_head_dump(buf, PAGE_SIZE, &charger->chg_data);
	if (len < PAGE_SIZE - 1)
		buf[len++] = '\n';

	len += p9221_adapter_capabilities_dump(&buf[len], PAGE_SIZE - len, &charger->chg_data);
	if (len < PAGE_SIZE - 1)
		buf[len++] = '\n';

	for (i = 0; i < WLC_SOC_STATS_LEN; i++) {
		if (charger->chg_data.soc_data[i].elapsed_time == 0)
			continue;
		len += p9221_soc_data_dump(&buf[len], PAGE_SIZE - len,
					   &charger->chg_data, i);
		if (len < PAGE_SIZE - 1)
			buf[len++] = '\n';
	}

enodata_done:
	mutex_unlock(&charger->stats_lock);
	return len;
}

static ssize_t p9221_ctl_chg_stats(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (count < 1)
		return -ENODATA;

	mutex_lock(&charger->stats_lock);
	switch (buf[0]) {
	case 0:
	case '0':
		p9221_charge_stats_init(&charger->chg_data);
		break;
	}
	mutex_unlock(&charger->stats_lock);

	return count;
}

static DEVICE_ATTR(charge_stats, 0644, p9221_show_chg_stats, p9221_ctl_chg_stats);

static int p9382_disable_dcin_en(struct p9221_charger_data *charger, bool enable)
{
	int ret;

	if (!charger->disable_dcin_en_votable)
		charger->disable_dcin_en_votable =
			gvotable_election_get_handle("DC_SUSPEND");
	if (!charger->disable_dcin_en_votable)
		return 0;

	ret = gvotable_cast_bool_vote(charger->disable_dcin_en_votable,
				      P9221_WLC_VOTER, enable);
	if (ret == 0)
		return 0;

	dev_err(&charger->client->dev, "Could not vote DISABLE_DCIN_EN (%d)\n", ret);
	return ret;
}

static bool p9xxx_rtx_gpio_is_state(struct p9221_charger_data *charger,
				    enum p9xxx_rtx_gpio_state state)
{
	int loops;
	bool ret;

	mutex_lock(&charger->rtx_gpio_lock);

	/* only wait when try to enable */
	if (state != RTX_READY)
		goto unlock;

	for (loops = 10 ; loops ; loops--) {
		if (charger->rtx_gpio_state != RTX_WAIT)
			break;

		mutex_unlock(&charger->rtx_gpio_lock);
		dev_dbg(&charger->client->dev, "RTx wait status\n");
		msleep(100);
		mutex_lock(&charger->rtx_gpio_lock);
	}
unlock:
	ret = charger->rtx_gpio_state == state;
	mutex_unlock(&charger->rtx_gpio_lock);
	return ret;
}

static void p9xxx_rtx_gpio_wait(struct p9221_charger_data *charger)
{
	mutex_lock(&charger->rtx_gpio_lock);

	charger->rtx_gpio_state = RTX_WAIT;
	if (!charger->pdata->has_rtx_gpio)
		charger->rtx_gpio_state = RTX_READY;

	mutex_unlock(&charger->rtx_gpio_lock);
}

static int p9xxx_rtx_mode_en(struct p9221_charger_data *charger, bool enable)
{
	if (!enable)
		return charger->chip_tx_mode_en(charger, false);

	if (p9xxx_rtx_gpio_is_state(charger, RTX_READY))
		return charger->chip_tx_mode_en(charger, true);

	return -ENOTSUPP;
}

/* requires mutex_lock(&charger->rtx_lock) when p9382_set_rtx() called */
static int p9382_set_rtx(struct p9221_charger_data *charger, bool enable)
{
	const bool rtx_gpio_retry = p9xxx_rtx_gpio_is_state(charger, RTX_RETRY);
	int ret = 0, tx_icl = -1;
	bool reset = false;

	if ((enable && charger->ben_state) || (!enable && !charger->ben_state)) {
		logbuffer_prlog(charger->rtx_log, "RTx is %s\n", enable ? "enabled" : "disabled");
		return 0;
	}

	if (charger->ben_state == FACTORY_BOOST_ENABLED) {
		logbuffer_prlog(charger->rtx_log, "RTx not allowed due to factory mode\n");
		return 0;
	}

	if (enable == 0) {
		if (charger->is_rtx_mode) {
			ret = p9xxx_rtx_mode_en(charger, false);
			charger->is_rtx_mode = false;
		}
		/* make sure it will re-enable so that vote can reset if error happens */
		if (!rtx_gpio_retry) {
			ret = p9382_ben_cfg(charger, RTX_BEN_DISABLED);
			if (ret < 0)
				goto error;

			ret = p9382_disable_dcin_en(charger, false);

			/* reset rtx gpio state */
			p9xxx_rtx_gpio_wait(charger);
		}
		logbuffer_log(charger->rtx_log, "disable rtx\n");

		goto done;
	} else {
		logbuffer_log(charger->rtx_log, "enable rtx");
		/* Check if there is any one vote disabled */
		if (charger->tx_icl_votable)
			tx_icl = gvotable_get_current_int_vote(
					charger->tx_icl_votable);
		if (tx_icl == 0) {
			dev_err(&charger->client->dev, "rtx be disabled\n");
			logbuffer_log(charger->rtx_log, "rtx be disabled\n");
			charger->rtx_reset_cnt = 0;
			reset = true;
		}

		/*
		 * Check if WLC online
		 * NOTE: when used CHARGER_MODE will also prevent this.
		 */
		if (charger->online) {
			dev_err(&charger->client->dev,
				"rTX is not allowed during WLC\n");
			logbuffer_log(charger->rtx_log,
				      "rTX is not allowed during WLC\n");
			charger->rtx_reset_cnt = 0;
			reset = true;
		}

		/*
		 * DCIN_EN votable will not be available on all systems.
		 * if it is there, it is needed.
		 */
		ret = p9382_disable_dcin_en(charger, !reset ? true : false);

		/*
		 * clear state and error by thermal or hardware limitation before enable
		 * but keep the unsupported type of error decided by state machine
		 */
		charger->com_busy = 0;
		charger->rtx_csp = 0;
		charger->rtx_err &= RTX_CHRG_NOTSUP_BIT;
		charger->is_rtx_mode = false;
		p9xxx_rtx_gpio_wait(charger);

		if (reset) {
			ret = p9382_ben_cfg(charger, RTX_BEN_DISABLED);
			goto done;
		}

		if (ret == 0)
			ret = p9382_ben_cfg(charger, RTX_BEN_ON);

		if (ret < 0) {
			dev_err(&charger->client->dev, "cannot enable rTX mode %d\n", ret);
			goto error;
		}

		msleep(10);

		ret = p9xxx_rtx_mode_en(charger, true);

		if (ret < 0) {
			dev_err(&charger->client->dev,
				"cannot enter rTX mode (%d)\n", ret);
			logbuffer_log(charger->rtx_log,
				      "cannot enter rTX mode (%d)\n", ret);
			p9382_ben_cfg(charger, RTX_BEN_DISABLED);

			ret = p9382_disable_dcin_en(charger, false);

			charger->is_rtx_mode = false;
			goto error;
		}

		ret = p9221_enable_interrupts(charger);
		if (ret)
			dev_err(&charger->client->dev,
				"Could not enable interrupts: %d\n", ret);

		/* configure TX_ICL */
		if (charger->tx_icl_votable)
			tx_icl = gvotable_get_current_int_vote(
					charger->tx_icl_votable);

		if ((tx_icl > 0) &&
		    (tx_icl != P9221_MA_TO_UA(P9382A_RTX_ICL_MAX_MA))) {
			ret = charger->chip_set_tx_ilim(charger, tx_icl);
			if (ret == 0)
				logbuffer_log(charger->rtx_log,
					      "set Tx current limit: %dmA",
					      tx_icl);
			else
				dev_err(&charger->client->dev,
					"Could not set Tx current limit: %d\n",
					ret);
		}
		goto done;
	}
error:
	if (charger->rtx_reset_cnt > 0) {
		charger->rtx_err |= RTX_HARD_OCP_BIT;
		charger->rtx_reset_cnt = 0;
	}
done:
	if (charger->rtx_reset_cnt == 0)
		schedule_work(&charger->uevent_work);

	if (enable && charger->is_rtx_mode && !charger->rtx_wakelock) {
		pm_stay_awake(charger->dev);
		charger->rtx_wakelock = true;
	} else if (!enable && charger->rtx_wakelock) {
		pm_relax(charger->dev);
		charger->rtx_wakelock = false;
	} else {
		dev_info(&charger->client->dev, "%s RTx(%d), rtx_wakelock=%d\n",
			 enable ? "enable" : "disable",
			 charger->is_rtx_mode, charger->rtx_wakelock);
	}
	dev_dbg(&charger->client->dev, "%s RTx(%d), rtx_wakelock=%d\n",
		enable ? "enable" : "disable", charger->is_rtx_mode, charger->rtx_wakelock);

	return ret;
}

#define P9412_RTX_OCP_THRES_MA		900
static int p9412_check_rtx_ocp(struct p9221_charger_data *chgr)
{
	int ret, cnt, retry, ocp_count = 0, current_now, chk_ms, total_delay;
	u16 status_reg;

	total_delay = chgr->rtx_total_delay > 0 ? chgr->rtx_total_delay : 3000;
	chk_ms = chgr->rtx_ocp_chk_ms > 0 ? chgr->rtx_ocp_chk_ms : 20;
	retry = (total_delay / chk_ms > 0) ? (total_delay / chk_ms) : 1;

	logbuffer_log(chgr->rtx_log, "use rtx_ocp_chk_ms=%d retry=%d", chk_ms, retry);

	for (cnt = 0; cnt < retry; cnt++) {
		if (!chgr->ben_state)
			return -EIO;
		/* check if rx_is_connected: if yes, goto 7V */
		ret = chgr->reg_read_16(chgr, P9221_STATUS_REG, &status_reg);
		if (ret < 0) {
			logbuffer_log(chgr->rtx_log, "ioerr disable RTx(%d)", ret);
			return -EIO;
		}
		if (status_reg & chgr->ints.rx_connected_bit) {
			logbuffer_log(chgr->rtx_log, "rx is connected, goto 7V");
			return 0;
		}
		/* check if (ocp_cnt > 2): if yes, disable rtx */
		ret = chgr->chip_get_iout(chgr, &current_now);
		if (ret == 0 && current_now > P9412_RTX_OCP_THRES_MA) {
			ocp_count++;
			logbuffer_log(chgr->rtx_log, "cnt=%d,current_now=%d,ocp_count=%d",
				      cnt, current_now, ocp_count);
		}
		if (ret < 0) {
			logbuffer_log(chgr->rtx_log, "iout disable RTx(%d)", ret);
			return -EINVAL;
		}
		if (ocp_count > 2 || current_now == 0) {
			logbuffer_log(chgr->rtx_log, "ocp_count=%d current_now=%d disable RTx",
				      ocp_count, current_now);
			return -EINVAL;
		}
		usleep_range(chk_ms * USEC_PER_MSEC, (chk_ms + 2) * USEC_PER_MSEC);
	}

	return 0;
}

/*
 * Check if Tx OCP occurs during ping phase before enabling 7V
 * acquires mutex_lock(&charger->rtx_lock) for calling p9382_set_rtx()
 */
static void p9412_chk_rtx_ocp_work(struct work_struct *work)
{
	struct p9221_charger_data *chgr = container_of(work,
			struct p9221_charger_data, chk_rtx_ocp_work.work);
	int ret;

	mutex_lock(&chgr->rtx_lock);
	if (!chgr->ben_state)
		goto done;

	/* check TX OCP before enable 7V */
	ret = p9412_check_rtx_ocp(chgr);
	if (ret < 0) {
		p9382_set_rtx(chgr, false);
		goto done;
	}

	ret = chgr->reg_write_8(chgr, P9412_APBSTPING_REG, P9412_APBSTPING_7V);
	ret |= chgr->reg_write_8(chgr, P9412_APBSTCONTROL_REG, P9412_APBSTPING_7V);
	if (ret == 0) {
		schedule_delayed_work(&chgr->rtx_work, msecs_to_jiffies(P9382_RTX_TIMEOUT_MS));
	} else {
		logbuffer_log(chgr->rtx_log, "Failed to configure Ext-Boost Vout registers(%d)",
			      ret);
		p9382_set_rtx(chgr, false);
	}
done:
	mutex_unlock(&chgr->rtx_lock);
}

static ssize_t rtx_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->ben_state);
}

/*
 * write 1 to enable boost & switch, write 0 to 0x34, wait for 0x4c==0x4
 * write 0 to write 0x80 to 0x4E, wait for 0x4c==0, disable boost & switch
 *
 * acquires mutex_lock(&charger->rtx_lock) for calling p9382_set_rtx()
 */
static ssize_t rtx_store(struct device *dev,
		       struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret;
	enum p9382_rtx_state rtx_status;

	if (!charger->bcl_wlc_votable)
		charger->bcl_wlc_votable = gvotable_election_get_handle(BCL_WLC);
	if (!charger->bcl_wlc_votable)
		logbuffer_prlog(charger->rtx_log, "can't find bcl votable");

	if (buf[0] == '0') {
		mutex_lock(&charger->rtx_lock);
		logbuffer_prlog(charger->rtx_log, "battery share off");
		if (charger->bcl_wlc_votable)
			gvotable_cast_vote(charger->bcl_wlc_votable, BCL_DEV_VOTER,
					   (void *)BCL_WLC_VOTE, WLC_DISABLED_TX);
		charger->rtx_reset_cnt = 0;
		ret = p9382_set_rtx(charger, false);
		mutex_unlock(&charger->rtx_lock);
	} else if (buf[0] == '1') {
		mutex_lock(&charger->rtx_lock);
		logbuffer_prlog(charger->rtx_log, "battery share on");
		if (charger->bcl_wlc_votable)
			gvotable_cast_vote(charger->bcl_wlc_votable, BCL_DEV_VOTER,
					   (void *)BCL_WLC_VOTE, WLC_ENABLED_TX);
		charger->rtx_reset_cnt = 0;
		rtx_status = p9xxx_get_rtx_status(charger);
		if (rtx_status == RTX_AVAILABLE)
			ret = p9382_set_rtx(charger, true);
		else if (rtx_status == RTX_ACTIVE)
			ret = 0;
		else
			ret = -ENODEV;
		mutex_unlock(&charger->rtx_lock);
	} else {
		return -EINVAL;
	}

	if (ret == 0)
		return count;
	else
		return ret;
}

static DEVICE_ATTR_RW(rtx);


static ssize_t has_wlc_dc_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->pdata->has_wlc_dc);
}

/* write 1 to enable boost & switch, write 0 to 0x34, wait for 0x4c==0x4
 * write 0 to write 0x80 to 0x4E, wait for 0x4c==0, disable boost & switch
 */
static ssize_t has_wlc_dc_store(struct device *dev,
		       struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	charger->pdata->has_wlc_dc = buf[0] == '1';
	return count;
}

static DEVICE_ATTR_RW(has_wlc_dc);

static ssize_t gpp_enhanced_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->pdata->gpp_enhanced);
}

/* write 1 to enable GPP15W feature
 * write 0 to disable GPP15W feature and skip Auth DC_ICL
 */
static ssize_t gpp_enhanced_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	charger->pdata->gpp_enhanced = buf[0] == '1';
	return count;
}

static DEVICE_ATTR_RW(gpp_enhanced);

static ssize_t authstart_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	const bool need_icl = charger->is_mfg_google || charger->mfg == WLC_MFG_108_FOR_GOOGLE;

	return scnprintf(buf, PAGE_SIZE, "%c\n", !need_icl || charger->set_auth_icl ? 'Y' : 'N');
}

static ssize_t authstart_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	const bool need_icl = charger->is_mfg_google || charger->mfg == WLC_MFG_108_FOR_GOOGLE;
	const ktime_t timeout = ms_to_ktime(WLCDC_DEBOUNCE_TIME_S * 1000);
	int ret = 0;

	if (buf[0] != '1')
		return -EINVAL;

	mutex_lock(&charger->auth_lock);

	if (charger->set_auth_icl || !need_icl)
		goto unlock;

	if (!charger->chip_is_calibrated(charger)) {
		cancel_work_sync(&charger->calibration_work);
		schedule_work(&charger->calibration_work);
		ret = -EINVAL;
		goto unlock;
	}

	ret = p9221_set_auth_dc_icl(charger, true);

	if (ret == 0) {
		charger->set_auth_icl = true;
		pm_stay_awake(charger->dev);
		alarm_start_relative(&charger->auth_dc_icl_alarm, timeout);
		schedule_delayed_work(&charger->auth_dc_icl_work,
				      msecs_to_jiffies(WLCDC_AUTH_CHECK_INIT_DELAY_MS));
	} else if (ret < 0) {
		dev_err(&charger->client->dev, "cannot set Auth ICL: %d\n", ret);
	}

unlock:
	mutex_unlock(&charger->auth_lock);

	return ret == 0 ? count : ret;
}

static DEVICE_ATTR_RW(authstart);

static ssize_t log_current_filtered_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->log_current_filtered);
}

/* write '1' to enable logging */
static ssize_t log_current_filtered_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	charger->log_current_filtered = buf[0] == '1';
	return count;
}

static DEVICE_ATTR_RW(log_current_filtered);

static ssize_t align_delta_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%d\n", charger->align_delta);
}

static ssize_t align_delta_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);
	int ret, delta;

	ret = kstrtoint(buf, 10, &delta);

	if (ret < 0)
		return ret;

	charger->align_delta = delta;

	return count;
}

static DEVICE_ATTR_RW(align_delta);

static struct attribute *rtx_attributes[] = {
	&dev_attr_rtx_sw.attr,
	&dev_attr_rtx_boost.attr,
	&dev_attr_rtx.attr,
	&dev_attr_rtx_status.attr,
	&dev_attr_is_rtx_connected.attr,
	&dev_attr_rx_lvl.attr,
	&dev_attr_rtx_err.attr,
	&dev_attr_fan_level.attr,
	NULL
};

static const struct attribute_group rtx_attr_group = {
	.attrs		= rtx_attributes,
};

static struct attribute *p9221_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_status.attr,
	&dev_attr_addr.attr,
	&dev_attr_count.attr,
	&dev_attr_data.attr,
	&dev_attr_ccreset.attr,
	&dev_attr_txbusy.attr,
	&dev_attr_txdone.attr,
	&dev_attr_txlen.attr,
	&dev_attr_rxlen.attr,
	&dev_attr_rxdone.attr,
	&dev_attr_icl_ramp_ua.attr,
	&dev_attr_icl_ramp_delay_ms.attr,
	&dev_attr_force_epp.attr,
	&dev_attr_dc_icl_bpp.attr,
	&dev_attr_dc_icl_epp.attr,
	&dev_attr_alignment.attr,
	&dev_attr_aicl_delay_ms.attr,
	&dev_attr_aicl_icl_ua.attr,
	&dev_attr_operating_freq.attr,
	&dev_attr_ptmc_id.attr,
	&dev_attr_ext_ben.attr,
	&dev_attr_qi_vbus_en.attr,
	&dev_attr_has_wlc_dc.attr,
	&dev_attr_gpp_enhanced.attr,
	&dev_attr_authstart.attr,
	&dev_attr_log_current_filtered.attr,
	&dev_attr_charge_stats.attr,
	&dev_attr_fw_rev.attr,
	&dev_attr_authtype.attr,
	&dev_attr_features.attr,
	&dev_attr_mitigate_threshold.attr,
	&dev_attr_wpc_ready.attr,
	&dev_attr_qien.attr,
	&dev_attr_align_delta.attr,
	&dev_attr_ldo_en.attr,
	NULL
};

static ssize_t p9221_rxdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->rx_buf[pos], size);
	charger->rx_done = false;
	return size;
}

static struct bin_attribute bin_attr_rxdata = {
	.attr = {
		.name = "rxdata",
		.mode = 0400,
	},
	.read = p9221_rxdata_read,
	.size = P9221R5_DATA_RECV_BUF_SIZE,
};

static ssize_t p9221_txdata_read(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(buf, &charger->tx_buf[pos], size);
	return size;
}

static ssize_t p9221_txdata_write(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t pos, size_t size)
{
	struct p9221_charger_data *charger;
	charger = dev_get_drvdata(container_of(kobj, struct device, kobj));

	memcpy(&charger->tx_buf[pos], buf, size);
	return size;
}

static struct bin_attribute bin_attr_txdata = {
	.attr = {
		.name = "txdata",
		.mode = 0600,
	},
	.read = p9221_txdata_read,
	.write = p9221_txdata_write,
	.size  = P9221R5_DATA_SEND_BUF_SIZE,
};

static struct bin_attribute *p9221_bin_attributes[] = {
	&bin_attr_txdata,
	&bin_attr_rxdata,
	NULL,
};

static const struct attribute_group p9221_attr_group = {
	.attrs		= p9221_attributes,
	.bin_attrs	= p9221_bin_attributes,
};

static void print_current_samples(struct p9221_charger_data *charger,
					u32 *iout_val, int count)
{
	int i;
	char temp[P9221R5_OVER_CHECK_NUM * 9 + 1] = { 0 };

	for (i = 0; i < count ; i++)
		scnprintf(temp + i * 9, sizeof(temp) - i * 9,
			  "%08x ", iout_val[i]);

	dev_info(&charger->client->dev, "OVER IOUT_SAMPLES: %s\n", temp);
}

/*
 * Number of times to poll the status to see if the current limit condition
 * was transient or not.
 */
static void p9221_over_handle(struct p9221_charger_data *charger,
			      u16 irq_src)
{
	u8 reason = 0;
	int i;
	int ret;
	int ovc_count = 0;
	u32 iout_val[P9221R5_OVER_CHECK_NUM] = { 0 };

	dev_err(&charger->client->dev, "Received OVER INT: %02x\n", irq_src);

	if (irq_src & charger->ints.over_volt_bit) {
		reason = P9221_EOP_OVER_VOLT;
		goto send_eop;
	}

	if (irq_src & charger->ints.over_temp_bit) {
		reason = P9221_EOP_OVER_TEMP;
		goto send_eop;
	}

	if ((irq_src & charger->ints.over_uv_bit) && !(irq_src & charger->ints.over_curr_bit))
		return;

	/* Overcurrent, reduce ICL and poll to absorb any transients */

	if (charger->dc_icl_votable) {
		int icl;

		icl = gvotable_get_current_int_vote(charger->dc_icl_votable);
		if (icl < 0) {
			dev_err(&charger->client->dev,
				"Failed to read ICL (%d)\n", icl);
		} else if (icl > OVC_BACKOFF_LIMIT) {
			icl -= OVC_BACKOFF_AMOUNT;

			ret = gvotable_cast_int_vote(charger->dc_icl_votable,
						     P9221_OCP_VOTER,
						     icl, true);
			dev_err(&charger->client->dev,
				"Reduced ICL to %d (%d)\n", icl, ret);
		}
	}

	reason = P9221_EOP_OVER_CURRENT;
	for (i = 0; i < P9221R5_OVER_CHECK_NUM; i++) {
		ret = p9221_clear_interrupts(charger,
					     irq_src & charger->ints.stat_limit_mask);
		msleep(50);
		if (ret)
			continue;

		ret = charger->chip_get_iout(charger, &iout_val[i]);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read iout[%d]: %d\n", i, ret);
			continue;
		} else {
			iout_val[i] = P9221_MA_TO_UA(iout_val[i]);

			if (iout_val[i] > OVC_THRESHOLD)
				ovc_count++;
		}

		ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &irq_src);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read status: %d\n", ret);
			continue;
		}

		if ((irq_src & charger->ints.over_curr_bit) == 0) {
			print_current_samples(charger, iout_val, i + 1);
			dev_info(&charger->client->dev,
				 "OVER condition %04x cleared after %d tries\n",
				 irq_src, i);
			return;
		}

		dev_err(&charger->client->dev,
			"OVER status is still %04x, retry\n", irq_src);
	}

	if (ovc_count < OVC_LIMIT) {
		print_current_samples(charger, iout_val,
				      P9221R5_OVER_CHECK_NUM);
		dev_info(&charger->client->dev,
			 "ovc_threshold=%d, ovc_count=%d, ovc_limit=%d\n",
			 OVC_THRESHOLD, ovc_count, OVC_LIMIT);
		return;
	}

send_eop:
	dev_err(&charger->client->dev,
		"OVER is %04x, sending EOP %d\n", irq_src, reason);

	ret = charger->chip_send_eop(charger, reason);
	if (ret)
		dev_err(&charger->client->dev,
			"Failed to send EOP %d: %d\n", reason, ret);
}

static void p9382_txid_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, txid_work.work);
	int ret = 0;
	bool attached = 0;
	u16 status_reg;
	const u16 rx_connected_bit = charger->ints.rx_connected_bit;

	/* check rx_is_connected */
	ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
	attached = (ret == 0) ? (status_reg & rx_connected_bit) : 0;
	if (!attached)
		return;

	if (charger->ints.pppsent_bit && charger->com_busy) {
		if (charger->com_busy >= COM_BUSY_MAX) {
			if (p9221_send_ccreset(charger) == 0)
				charger->com_busy = 0;
		} else {
			charger->com_busy += 1;
		}
		schedule_delayed_work(&charger->txid_work,
				      msecs_to_jiffies(TXID_SEND_DELAY_MS));
		logbuffer_log(charger->rtx_log,
			      "com_busy=%d, reschedule txid_work()",
			      charger->com_busy);
		return;
	}

	ret = charger->chip_send_txid(charger);
	if (!ret) {
		p9221_hex_str(&charger->tx_buf[1], FAST_SERIAL_ID_SIZE,
			      charger->fast_id_str,
			      sizeof(charger->fast_id_str), false);
		dev_info(&charger->client->dev, "Fast serial ID send(%s)\n",
			 charger->fast_id_str);
		charger->com_busy += 1;
		if (charger->send_txid_cnt > 0) {
			charger->send_txid_cnt--;
			schedule_delayed_work(&charger->txid_work,
					      msecs_to_jiffies(TXID_SEND_AGAIN_DELAY_MS));
		}
	}
}

/* requires mutex_lock(&charger->rtx_lock); */
static void p9xxx_reset_rtx(struct p9221_charger_data *charger)
{
	const bool rtx_gpio_retry = p9xxx_rtx_gpio_is_state(charger, RTX_RETRY);
	int ext_bst_on = 0;

	if (!rtx_gpio_retry) {
		charger->rtx_reset_cnt += 1;

		if (charger->rtx_reset_cnt >= RTX_RESET_COUNT_MAX) {
			if (charger->rtx_reset_cnt == RTX_RESET_COUNT_MAX)
				charger->rtx_err |= RTX_HARD_OCP_BIT;
			charger->rtx_reset_cnt = 0;
		}
	} else {
		charger->rtx_reset_cnt = 0;
	}
	charger->is_rtx_mode = false;
	/* use gpio_get to log any plc errors during rtx */
	if (charger->pdata->ben_gpio > 0)
		gpio_get_value_cansleep(charger->pdata->ben_gpio);
	p9382_set_rtx(charger, false);

	msleep(REENABLE_RTX_DELAY);

	/* external boost is on but not for rtx */
	if (charger->pdata->ben_gpio > 0)
		ext_bst_on = gpio_get_value_cansleep(charger->pdata->ben_gpio);
	if (ext_bst_on && !rtx_gpio_retry) {
		dev_warn(&charger->client->dev, "not allowed to re-enable due to ext on");
		schedule_work(&charger->uevent_work);
		return;
	}

	if (charger->rtx_reset_cnt || rtx_gpio_retry) {
		dev_info(&charger->client->dev, "re-enable RTx mode, cnt=%d\n", charger->rtx_reset_cnt);
		logbuffer_log(charger->rtx_log, "re-enable RTx mode, cnt=%d\n", charger->rtx_reset_cnt);
		p9382_set_rtx(charger, true);
		schedule_work(&charger->uevent_work);
	}
}

/*
 * This will be called from interrupt handler
 * acquires mutex_lock(&charger->rtx_lock) for calling p9382_set_rtx()
 */
static void p9xxx_rtx_reset_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, rtx_reset_work);

	mutex_lock(&charger->rtx_lock);
	/* Skip if RTx is turned off from UI */
	if (!charger->ben_state)
		goto unlock_done;

	p9xxx_reset_rtx(charger);

unlock_done:
	mutex_unlock(&charger->rtx_lock);
}

/*
 * This is monitor system mode when RTx is enabled
 * acquires mutex_lock(&charger->rtx_lock) for calling p9382_set_rtx()
 */
static void p9382_rtx_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, rtx_work.work);
	const bool rtx_gpio_retry = p9xxx_rtx_gpio_is_state(charger, RTX_RETRY);
	u8 mode_reg = 0;
	int ret = 0;

	mutex_lock(&charger->rtx_lock);
	/* Skip if RTx is turned off from UI */
	if (!charger->ben_state)
		goto unlock_done;

	/* Check if RTx mode is auto turn off */
	ret = charger->chip_get_sys_mode(charger, &mode_reg);
	if (ret == 0 &&
	    mode_reg & P9XXX_SYS_OP_MODE_TX_MODE &&
	    !rtx_gpio_retry)
		goto reschedule;

	dev_info(&charger->client->dev, "is_rtx_on: ben=%d, mode=%02x, ret=%d",
		 charger->ben_state, mode_reg, ret);
	logbuffer_log(charger->rtx_log, "is_rtx_on: ben=%d, mode=%02x, ret=%d",
		      charger->ben_state, mode_reg, ret);

	p9xxx_reset_rtx(charger);

reschedule:
	schedule_delayed_work(&charger->rtx_work,
			      msecs_to_jiffies(P9382_RTX_TIMEOUT_MS));
unlock_done:
	mutex_unlock(&charger->rtx_lock);
}

/* Handler for rtx mode */
static void rtx_irq_handler(struct p9221_charger_data *charger, u16 irq_src)
{
	int ret;
	u8 mode_reg, csp_reg;
	u16 status_reg;
	bool attached = 0;
	const u16 mode_changed_bit = charger->ints.mode_changed_bit;
	const u16 pppsent_bit = charger->ints.pppsent_bit;
	const u16 hard_ocp_bit = charger->ints.hard_ocp_bit;
	const u16 tx_conflict_bit = charger->ints.tx_conflict_bit;
	const u16 rx_connected_bit = charger->ints.rx_connected_bit;
	const u16 csp_bit = charger->ints.csp_bit;
	const u16 ocp_ping_bit = charger->ints.ocp_ping_bit;

	if (irq_src & mode_changed_bit) {
		ret = charger->chip_get_sys_mode(charger, &mode_reg);
		if (ret) {
			dev_err(&charger->client->dev,
				"Failed to read P9221_SYSTEM_MODE_REG: %d\n",
				ret);
			return;
		}
		if (mode_reg == P9XXX_SYS_OP_MODE_TX_MODE) {
			charger->is_rtx_mode = true;
			cancel_delayed_work_sync(&charger->rtx_work);
			if (!charger->pdata->apbst_en || charger->pdata->hw_ocp_det)
				schedule_delayed_work(&charger->rtx_work,
						      msecs_to_jiffies(P9382_RTX_TIMEOUT_MS));
		}
		dev_info(&charger->client->dev,
			 "P9221_SYSTEM_MODE_REG reg: %02x\n",
			 mode_reg);
		logbuffer_log(charger->rtx_log,
			      "SYSTEM_MODE_REG=%02x", mode_reg);
	}

	ret = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
	if (ret) {
		dev_err(&charger->client->dev,
			"failed to read P9221_STATUS_REG reg: %d\n",
			ret);
		return;
	}

	if (irq_src & pppsent_bit)
		charger->com_busy = 0;

	if (irq_src & (hard_ocp_bit | tx_conflict_bit | ocp_ping_bit)) {
		if (irq_src & hard_ocp_bit)
			charger->rtx_err |= RTX_HARD_OCP_BIT;
		else if (irq_src & tx_conflict_bit)
			charger->rtx_err |= RTX_TX_CONFLICT_BIT;

		logbuffer_prlog(charger->rtx_log, "rtx_err=%d, STATUS_REG=%04x",
				charger->rtx_err, status_reg);

		cancel_delayed_work_sync(&charger->rtx_work);
		if (charger->rtx_err & RTX_HARD_OCP_BIT) {
		        /* keep unsupported error decided by state machine */
			charger->rtx_err &= RTX_CHRG_NOTSUP_BIT;
			schedule_work(&charger->rtx_reset_work);
		} else {
			charger->is_rtx_mode = false;
			schedule_work(&charger->rtx_disable_work);
		}
	}

	if (irq_src & rx_connected_bit) {
		attached = status_reg & rx_connected_bit;
		logbuffer_log(charger->rtx_log,
			      "Rx is %s. STATUS_REG=%04x",
			      attached ? "connected" : "disconnect",
			      status_reg);
		schedule_work(&charger->uevent_work);
		if (attached) {
			cancel_delayed_work_sync(&charger->txid_work);
			charger->send_txid_cnt = 2;
			schedule_delayed_work(&charger->txid_work,
					msecs_to_jiffies(TXID_SEND_DELAY_MS));
		} else {
			cancel_delayed_work_sync(&charger->txid_work);
			charger->send_txid_cnt = 0;
			charger->rtx_csp = 0;
			charger->com_busy = 0;
		}
	}

	if (irq_src & csp_bit) {
		ret = p9221_reg_read_8(charger, charger->reg_csp_addr, &csp_reg);
		if (ret) {
			logbuffer_log(charger->rtx_log, "failed to read CSP_REG reg: %d", ret);
		} else {
			charger->rtx_csp = csp_reg;
			schedule_work(&charger->uevent_work);
		}
	}
}


#ifdef CONFIG_DC_RESET
/*
 * DC reset code uses a flag in the charger to initiate a hard reset of the
 * WLC chip after a power loss. This is (was?) needed for p9221 to handle
 * partial and/or rapid entry/exit from the field that could cause firmware
 * to become erratic.
 */
static bool p9221_dc_reset_needed(struct p9221_charger_data *charger,
				  u16 irq_src)
{
	/*
	 * It is suspected that p9221 misses to set the interrupt status
	 * register occasionally. Evaluate spurious interrupt case for
	 * dc reset as well.
	 */
	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_MODECHANGED &&
	    (irq_src & charger->ints.mode_changed_bit || !irq_src)) {
		u8 mode_reg = 0;
		int res;

		res = charger->chip_get_sys_mode(charger, &mode_reg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"Failed to read P9221_SYSTEM_MODE_REG: %d\n",
				res);
			/*
			 * p9221_reg_read_n returns ENODEV for ENOTCONN as well.
			 * Signal dc_reset when register read fails with the
			 * above reasons.
			 */
			return res == -ENODEV;
		}

		dev_info(&charger->client->dev,
			 "P9221_SYSTEM_MODE_REG reg: %02x\n", mode_reg);
		return !(mode_reg == P9XXX_SYS_OP_MODE_WPC_EXTD ||
			 mode_reg == P9XXX_SYS_OP_MODE_PROPRIETARY ||
			 mode_reg == P9XXX_SYS_OP_MODE_WPC_BASIC);
	}

	if (charger->pdata->needs_dcin_reset == P9221_WC_DC_RESET_VOUTCHANGED &&
	    irq_src & charger->ints.vout_changed_bit) {
		u16 status_reg = 0;
		int res;

		res = p9221_reg_read_16(charger, P9221_STATUS_REG, &status_reg);
		if (res < 0) {
			dev_err(&charger->client->dev,
				"Failed to read P9221_STATUS_REG: %d\n", res);
			return res == -ENODEV ? true : false;
		}

		dev_info(&charger->client->dev,
			 "P9221_STATUS_REG reg: %04x\n", status_reg);
		return !(status_reg & charger->ints.vout_changed_bit);
	}

	return false;
}

static void p9221_check_dc_reset(struct p9221_charger_data *charger,
				    u16 irq_src)
{
	union power_supply_propval val = {.intval = 1};
	int res;

	if (!p9221_dc_reset_needed(charger, irq_src))
		return;

	if (!charger->dc_psy)
		charger->dc_psy = power_supply_get_by_name("dc");
	if (charger->dc_psy) {
		/* Signal DC_RESET when wireless removal is sensed. */
		res = power_supply_set_property(charger->dc_psy,
					POWER_SUPPLY_PROP_DC_RESET,
					&val);
	} else {
		res = -ENODEV;
	}

	if (res < 0)
		dev_err(&charger->client->dev,
			"unable to set DC_RESET, ret=%d",
			res);
}
#else
static void p9221_check_dc_reset(struct p9221_charger_data *charger,
				 u16 irq_src)
{

}
#endif

static void p9221_handle_pp(struct p9221_charger_data *charger)
{
	u8 tmp;
	u8 buff[sizeof(charger->pp_buf)];
	char bufstr[sizeof(charger->pp_buf) * 3 + 1];
	int msg_len;
	int res;

	res = p9xxx_chip_get_pp_buf(charger, buff, sizeof(buff));
	if (res) {
		dev_err(&charger->client->dev, "Failed to read PP: %d\n", res);
		return;
	}

	/* WPC 1.2.4: 5.2.2.4.1 */
	switch (buff[0]) {
	case 0x00 ... 0x1F:
		msg_len = 1 + (buff[0] - 0) / 32;
		break;
	case 0x20 ... 0x7F:
		msg_len = 2 + (buff[0] - 32) / 16;
		break;
	case 0x80 ... 0xDF:
		msg_len = 8 + (buff[0] - 128) / 8;
		break;
	case 0xE0 ... 0xFF:
		msg_len = 20 + (buff[0] - 224) / 4;
		break;
	}

	/* len is the length of the data + 1 for header. (cksum not supplied) */
	p9221_hex_str(buff, msg_len + 1, bufstr, sizeof(bufstr), false);
	dev_dbg(&charger->client->dev, "Received PP: %s\n", bufstr);

	if ((buff[0] == CHARGE_STATUS_PACKET_HEADER) &&
	    (buff[1] == PP_TYPE_POWER_CONTROL) &&
	    (buff[2] == PP_SUBTYPE_SOC)) {
		u8 crc = p9221_crc8(&buff[1], CHARGE_STATUS_PACKET_SIZE - 1,
				    CRC8_INIT_VALUE);
		if (buff[4] != crc) {
			dev_err(&charger->client->dev, "PP CSP CRC mismatch\n");
			return;
		}
		charger->rtx_csp = buff[3] / 2;
		dev_info(&charger->client->dev, "Received Tx's soc=%d\n",
			 charger->rtx_csp);
		schedule_work(&charger->uevent_work);
		return;
	}

	/*
	 * We only care about 0x4F proprietary packets.  Don't touch pp_buf
	 * if there is no match.
	 */
	if (buff[0] != 0x4f)
		return;
	logbuffer_log(charger->log, "Received PP: %s", bufstr);
	memcpy(charger->pp_buf, buff, sizeof(charger->pp_buf));
	charger->pp_buf_valid = 1;

	/* Check if charging on a Tx phone */
	tmp = charger->pp_buf[4] & ACCESSORY_TYPE_MASK;
	charger->chg_on_rtx = (tmp == ACCESSORY_TYPE_PHONE);
	dev_info(&charger->client->dev, "chg_on_rtx=%d\n", charger->chg_on_rtx);
	if (charger->chg_on_rtx) {
		gvotable_cast_long_vote(charger->dc_icl_votable,
					P9382A_RTX_VOTER,
					P9221_DC_ICL_RTX_UA, true);
		dev_info(&charger->client->dev, "set ICL to %dmA",
			 P9221_DC_ICL_RTX_UA / 1000);
	}
	/* check phone type bit[17-23] */
	tmp = (charger->pp_buf[3] & TXID_PHONE_TYPE_MASK) >> 1;
	charger->chg_on_cc_rtx = tmp == TXID_PHONE_TYPE_CC;
	if (charger->chg_on_cc_rtx) {
		res = p9xxx_chip_set_cmfet_reg(charger, RA9530_CMFET_COM_1_B);
		dev_info(&charger->client->dev, "change comm cap to CM1 + CMB (0x%02x), ret=%d\n",
			RA9530_CMFET_COM_1_B, res);
	}
}

/* Handler for R5 and R7 chips */
static void p9221_irq_handler(struct p9221_charger_data *charger, u16 irq_src)
{
	int res;

	p9221_check_dc_reset(charger, irq_src);

	if (irq_src & charger->ints.stat_limit_mask)
		p9221_over_handle(charger, irq_src);

	/* Receive complete */
	if (irq_src & charger->ints.cc_data_rcvd_bit) {
		size_t rxlen = 0;

		res = charger->chip_get_cc_recv_size(charger, &rxlen);
		if (res) {
			dev_err(&charger->client->dev,
				"Failed to read len: %d\n", res);
			rxlen = 0;
		}
		if (rxlen) {
			res = charger->chip_get_data_buf(charger,
							 charger->rx_buf,
							 rxlen);
			if (res)
				dev_err(&charger->client->dev,
					"Failed to read len: %d\n", res);

			charger->rx_len = rxlen;
			charger->rx_done = true;
			charger->cc_data_lock.cc_rcv_at = get_boot_msec();
			set_renego_state(charger, P9XXX_AVAILABLE);
			sysfs_notify(&charger->dev->kobj, NULL, "rxdone");
		}
	}

	/* Send complete */
	if (irq_src & charger->ints.cc_send_busy_bit) {
		charger->tx_busy = false;
		charger->tx_done = true;
		charger->cc_data_lock.cc_use = true;
		charger->cc_data_lock.cc_rcv_at = 0;
		if (charger->cc_reset_pending) {
			charger->cc_data_lock.cc_use = false;
			charger->cc_reset_pending = false;
			wake_up_all(&charger->ccreset_wq);
		}
		cancel_delayed_work(&charger->tx_work);
		sysfs_notify(&charger->dev->kobj, NULL, "txbusy");
		sysfs_notify(&charger->dev->kobj, NULL, "txdone");
	}

	/* Proprietary packet */
	if (irq_src & charger->ints.pp_rcvd_bit) {
		p9221_handle_pp(charger);
	}

	/* CC Reset complete */
	if (irq_src & charger->ints.cc_reset_bit)
		p9221_abort_transfers(charger);

	if (irq_src & charger->ints.propmode_stat_bit) {
		u8 mode;

		res = charger->chip_get_sys_mode(charger, &mode);
		if (res == 0 && mode == P9XXX_SYS_OP_MODE_PROPRIETARY) {
			/*
			 * If proprietary mode is enabled, indicates a pending interrupt
			 * when the negotiation process is complete.
			 */
			if (charger->prop_mode_en == true)
				charger->negotiation_complete = true;
			charger->prop_mode_en = true;
		}

		/* charger->prop_mode_en is reset on disconnect */
	}

	/* This only necessary for P9222 */
	if (irq_src & charger->ints.extended_mode_bit) {
		charger->extended_int_recv = true;
		if (charger->check_rp == RP_NOTSET &&
		    charger->pdata->epp_rp_value != -1) {
			pm_stay_awake(charger->dev);
			charger->check_rp = RP_CHECKING;
			cancel_delayed_work_sync(&charger->chk_rp_work);
			schedule_delayed_work(&charger->chk_rp_work,
					      msecs_to_jiffies(P9XXX_CHK_RP_DELAY_MS));
		}
		schedule_delayed_work(&charger->chk_fod_work, 0);
	}
}

#define IRQ_DEBOUNCE_TIME_MS		4
static irqreturn_t p9221_irq_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;
	int ret;
	u16 irq_src = 0;
	u8 sys_mode;
	ktime_t now = get_boot_msec();

	if ((now - charger->irq_at) < IRQ_DEBOUNCE_TIME_MS)
		return IRQ_HANDLED;

	charger->irq_at = now;

	pm_runtime_get_sync(charger->dev);
	if (!charger->resume_complete) {
		dev_warn(charger->dev, "%s: irq skipped, irq:%d\n", __func__, irq);
		pm_runtime_put_sync(charger->dev);
		return IRQ_HANDLED;
	}
	pm_runtime_put_sync(charger->dev);

	ret = p9221_reg_read_16(charger, P9221_INT_REG, &irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to read INT reg: %d\n", ret);
		goto out;
	}

	/* TODO: interrupt storm with irq_src = when in rTX mode */
	if (!charger->ben_state) {
		dev_info(&charger->client->dev, "INT: %04x\n", irq_src);
		logbuffer_log(charger->log, "INT=%04x on:%d",
			      irq_src, charger->online);
	}

	if (!irq_src)
		goto out;

	ret = p9221_clear_interrupts(charger, irq_src);
	if (ret) {
		dev_err(&charger->client->dev,
			"Failed to clear INT reg: %d\n", ret);
		goto out;
	}

	if (charger->ben_state == FACTORY_BOOST_ENABLED)
		goto out;

	/* todo interrupt handling for rx */
	if (charger->ben_state) {
		logbuffer_log(charger->rtx_log, "INT=%04x", irq_src);
		rtx_irq_handler(charger, irq_src);
		goto out;
	}

	if (irq_src & charger->ints.vrecton_bit) {
		dev_info(&charger->client->dev,
			"Received VRECTON, online=%d\n", charger->online);

		charger->chip_set_ovp(charger, OVSET_EPP);

		if (!charger->online) {
			charger->check_det = true;
			pm_stay_awake(charger->dev);

			if (charger->pdata->rf_value != -1)
				mod_delayed_work(system_wq, &charger->set_rf_work,
						 msecs_to_jiffies(P9XXX_SET_RF_DELAY_MS));
			if (!schedule_delayed_work(&charger->notifier_work,
				msecs_to_jiffies(P9221_NOTIFIER_DELAY_MS))) {
				pm_relax(charger->dev);
			}
		}
	}

	if (irq_src & charger->ints.vout_changed_bit) {
		ret = charger->chip_get_sys_mode(charger, &sys_mode);
		if (ret)
			dev_err(&charger->client->dev, "Failed to get mode to set ovp: %d\n", ret);
		else if (sys_mode == P9XXX_SYS_OP_MODE_WPC_EXTD)
			charger->chip_set_ovp(charger, OVSET_EPP);
		else if (sys_mode == P9XXX_SYS_OP_MODE_WPC_BASIC)
			charger->chip_set_ovp(charger, OVSET_BPP);
	}

	p9221_irq_handler(charger, irq_src);

out:
	return IRQ_HANDLED;
}

/*
 * This is a timeout function to disable the online_spoof when WLC gets re-enabled. It will
 * only be called in the event that DCIN doesn't come back for 2 seconds even though WLC is
 * no longer disabled.
 */
static void p9xxx_stop_online_spoof_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, stop_online_spoof_work.work);

	mutex_lock(&charger->irq_det_lock);
	if (charger->online_spoof) {
		/* timeout after WLC re-enabled */
		logbuffer_prlog(charger->log, "timeout: online_spoof=0");
		disable_irq_wake(charger->pdata->irq_det_int);
		disable_irq(charger->pdata->irq_det_int);
		charger->online_spoof = false;
		gvotable_cast_bool_vote(charger->wlc_spoof_votable, "WLC", false);
		power_supply_changed(charger->wc_psy);
	}
	mutex_unlock(&charger->irq_det_lock);
}

static void p9xxx_change_det_status_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, change_det_status_work.work);
	const int det_gpio = gpio_get_value_cansleep(charger->pdata->irq_det_gpio);

	/* Debounce det status */
	logbuffer_log(charger->log, "irq_det debounce: val=%d", det_gpio);
	mutex_lock(&charger->irq_det_lock);
	if (charger->det_status != det_gpio) {
		if (det_gpio == 1 && charger->det_status == 0 && charger->online_spoof) {
			logbuffer_prlog(charger->log, "det=0: online_spoof=0");
			disable_irq_wake(charger->pdata->irq_det_int);
			disable_irq(charger->pdata->irq_det_int);
			charger->online_spoof = false;
                        gvotable_cast_bool_vote(charger->wlc_spoof_votable, "WLC", false);
		}
		charger->det_status = det_gpio;
		power_supply_changed(charger->wc_psy);
	} else if (det_gpio == 1) {
		logbuffer_prlog(charger->log, "det=1->1: online_spoof=1");
		schedule_delayed_work(&charger->stop_online_spoof_work, 0);
	}
	mutex_unlock(&charger->irq_det_lock);
	__pm_relax(charger->det_status_ws);
}

static irqreturn_t p9221_irq_det_thread(int irq, void *irq_data)
{
	struct p9221_charger_data *charger = irq_data;
	const int det_gpio = gpio_get_value_cansleep(charger->pdata->irq_det_gpio);
	int debounce_interval = -1;

	logbuffer_log(charger->log, "irq_det: value=%d, status=%d, online=%d ben=%d",
			det_gpio, charger->det_status, charger->online, charger->ben_state);
	if (!charger->online_spoof) {
		charger->det_status = det_gpio;
	} else if (det_gpio == 1 && charger->det_status == 0) {
		debounce_interval = msecs_to_jiffies(charger->det_off_debounce);
	} else if (det_gpio == 0 && charger->det_status == 1) {
		debounce_interval = msecs_to_jiffies(charger->det_on_debounce);
	} else if (det_gpio == charger->det_status) {
		cancel_delayed_work(&charger->change_det_status_work);
		__pm_relax(charger->det_status_ws);
	}

	if (debounce_interval >= 0) {
		__pm_stay_awake(charger->det_status_ws);
		mod_delayed_work(system_wq, &charger->change_det_status_work, debounce_interval);
	}

	/* If we are already online, just ignore the interrupt. */
	if (p9221_is_online(charger))
		return IRQ_HANDLED;

	if (charger->align != WLC_ALIGN_MOVE) {
		if (charger->align != WLC_ALIGN_CHECKING)
			schedule_work(&charger->uevent_work);
		charger->align = WLC_ALIGN_CHECKING;
		charger->align_count++;

		if (charger->align_count > WLC_ALIGN_IRQ_THRESHOLD) {
			schedule_work(&charger->uevent_work);
			charger->align = WLC_ALIGN_MOVE;
		}
		logbuffer_log(charger->log, "align: state: %s",
			      align_status_str[charger->align]);
	}

	del_timer(&charger->align_timer);

	/*
	 * This interrupt will wake the device if it's suspended,
	 * but it is not reliable enough to trigger the charging indicator.
	 * Give ourselves 2 seconds for the VRECTON interrupt to appear
	 * before we put up the charging indicator.
	 */
	mod_timer(&charger->vrect_timer,
		  jiffies + msecs_to_jiffies(P9221_VRECT_TIMEOUT_MS));
	pm_stay_awake(charger->dev);

	return IRQ_HANDLED;
}

static void p9xxx_set_rf_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, set_rf_work.work);

	if (!charger->online)
		return;

	p9xxx_write_resonance_freq(charger);

	p9xxx_update_q_factor(charger);
}

static void p9xxx_chk_fod_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, chk_fod_work.work);

	if (!charger->online)
		return;

	p9221_write_fod(charger);
}

static void p9xxx_chk_rp_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, chk_rp_work.work);

	charger->chip_renegotiate_pwr(charger);
	pm_relax(charger->dev);
}

/*
 * This will be called from callback or interrupt handler for disable RTx
 * acquires mutex_lock(&charger->rtx_lock) for calling p9382_set_rtx()
 */
static void p9382_rtx_disable_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, rtx_disable_work);
	int ret = 0;

	mutex_lock(&charger->rtx_lock);
	if (!charger->bcl_wlc_votable)
		charger->bcl_wlc_votable = gvotable_election_get_handle(BCL_WLC);
	if (charger->bcl_wlc_votable)
		gvotable_cast_vote(charger->bcl_wlc_votable, BCL_DEV_VOTER,
				   (void *)BCL_WLC_VOTE, WLC_DISABLED_TX);

	charger->rtx_reset_cnt = 0;

	/* Disable rtx mode */
	ret = p9382_set_rtx(charger, false);
	if (ret)
		dev_err(&charger->client->dev,
			"unable to disable rtx: %d\n", ret);
	mutex_unlock(&charger->rtx_lock);
}

/* send out a uevent notification and log iout/vout */
static void p9221_uevent_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, uevent_work);
	int ret;
	u32 vout, iout;

	kobject_uevent(&charger->dev->kobj, KOBJ_CHANGE);

	if (!charger->ben_state)
		return;

	ret = charger->chip_get_iout(charger, &iout);
	ret |= charger->chip_get_vout(charger, &vout);
	if (ret == 0) {
		logbuffer_log(charger->rtx_log,
			      "Vout=%umV, Iout=%umA, rx_lvl=%u",
			      vout, iout,
			      charger->rtx_csp);
	} else {
		logbuffer_log(charger->rtx_log, "failed to read rtx info.");
	}
}

static void p9xxx_calibration_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, calibration_work);

	for (int i = 0; i < 5; i++) {
		if (!p9221_is_epp(charger))
			break;
		dev_dbg(&charger->client->dev, "not calibrated yet, check again in 5 secs\n");
		msleep(5000);
		if (charger->chip_is_calibrated(charger)) {
			if (!charger->set_auth_icl)
				schedule_work(&charger->uevent_work);
			break;
		}
	}

}

static void p9221_parse_fod(struct device *dev,
			    int *fod_num, u8 *fod, char *of_name)
{
	int ret;
	struct device_node *node = dev->of_node;

	*fod_num = of_property_count_elems_of_size(node, of_name, sizeof(u8));
	if (*fod_num <= 0) {
		dev_err(dev, "No dt %s provided (%d)\n", of_name, *fod_num);
		*fod_num = 0;
	} else {
		if (*fod_num > P9221R5_NUM_FOD) {
			dev_err(dev,
			    "Incorrect num of %s: %d, using first %d\n",
			    of_name, *fod_num, P9221R5_NUM_FOD);
			*fod_num = P9221R5_NUM_FOD;
		}
		ret = of_property_read_u8_array(node, of_name, fod, *fod_num);
		if (ret == 0) {
			char buf[P9221R5_NUM_FOD * 3 + 1];

			p9221_hex_str(fod, *fod_num, buf, *fod_num * 3 + 1, false);
			dev_info(dev, "dt %s: %s (%d)\n", of_name, buf, *fod_num);
		}
	}
}

static int p9221_parse_hpp_fods(struct device *dev,
				struct p9221_charger_platform_data *pdata,
				int nb_hpp_fod_vol)
{
	int i;

	for (i = 0; i < nb_hpp_fod_vol; i++) {
		char of_name[36];

		scnprintf(of_name, sizeof(of_name), "fod_hpp_%d", i);
		p9221_parse_fod(dev, &pdata->hpp_fods[i].num,
				pdata->hpp_fods[i].fod, of_name);
	}
	if (nb_hpp_fod_vol != i) {
		dev_info(dev, "dt fod not match nb=%d i=%d\n", nb_hpp_fod_vol, i);
		return -EINVAL;
	}

	return 0;
}

static int p9221_parse_gpios(struct device *dev, char *of_name_1, char *of_name_2,
			     enum of_gpio_flags *flags)
{
	int ret_1, ret_2 = 0;
	struct device_node *node = dev->of_node;

	ret_1 = of_get_named_gpio_flags(node, of_name_1, 0, flags);
	if (ret_1 < 0)
		ret_2 = of_get_named_gpio_flags(node, of_name_2, 0, flags);

	if (ret_1 == -EPROBE_DEFER || ret_2 == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (ret_1 > 0)
		return ret_1;
	if (ret_2 > 0)
		return ret_2;

	dev_dbg(dev, "unable to read %s from dt: %d\n", of_name_1, ret_1);
	dev_dbg(dev, "unable to read %s from dt: %d\n", of_name_2, ret_2);

	return -EINVAL;
}

static int p9221_parse_dt(struct device *dev,
			  struct p9221_charger_platform_data *pdata)
{
	int ret = 0;
	u32 data;
	struct device_node *node = dev->of_node;
	int vout_set_max_mv = P9221_VOUT_SET_MAX_MV;
	int vout_set_min_mv = P9221_VOUT_SET_MIN_MV;
	int nb_hpp_fod_vol;
	enum of_gpio_flags flags = 0;

	if (of_device_is_compatible(node, "idt,p9412")) {
		dev_info(dev, "selecting p9412\n");
		pdata->chip_id = P9412_CHIP_ID;
		vout_set_min_mv = P9412_VOUT_SET_MIN_MV;
		vout_set_max_mv = P9412_VOUT_SET_MAX_MV;
	} else if (of_device_is_compatible(node, "idt,p9382")) {
		dev_info(dev, "selecting p9382\n");
		pdata->chip_id = P9382A_CHIP_ID;
	} else if (of_device_is_compatible(node, "idt,p9221")) {
		dev_info(dev, "selecting p9221\n");
		pdata->chip_id = P9221_CHIP_ID;
	} else if (of_device_is_compatible(node, "idt,p9222")) {
		dev_info(dev, "selecting p9222\n");
		pdata->chip_id = P9222_CHIP_ID;
	} else if (of_device_is_compatible(node, "idt,ra9530")) {
		dev_info(dev, "selecting ra9530\n");
		pdata->chip_id = RA9530_CHIP_ID;
	}

	/* QI_EN_L: enable/disable WLC chip */
	pdata->qien_gpio = p9221_parse_gpios(dev, "idt,qien-gpio", "idt,gpio_qien", &flags);
	if (pdata->qien_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->qien_gpio > 0)
		dev_info(dev, "enable gpio:%d", pdata->qien_gpio);

	/*
	 * QI_USB_VBUS_EN: control the priority of USB and WLC,
	 *                 set to high after boot
	 */
	pdata->qi_vbus_en = p9221_parse_gpios(dev, "idt,qi_vbus_en-gpio", "idt,gpio_qi_vbus_en",
					      &flags);
	if (pdata->qi_vbus_en == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->qi_vbus_en > 0) {
		pdata->qi_vbus_en_act_low = (flags & OF_GPIO_ACTIVE_LOW) != 0;
		dev_info(dev, "QI_USB_VBUS_EN gpio:%d(act_low=%d)",
			 pdata->qi_vbus_en, pdata->qi_vbus_en_act_low);
	}

	/* Enable/Disable WLC chip(for P9XXX_GPIO_VBUS_EN) */
	pdata->wlc_en = p9221_parse_gpios(dev, "idt,wlc_en-gpio", "idt,gpio_wlc_en", &flags);
	if (pdata->wlc_en == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->wlc_en > 0) {
		pdata->wlc_en_act_low = (flags & OF_GPIO_ACTIVE_LOW) != 0;
		dev_info(dev, "WLC enable/disable pin:%d(act_low=%d)",
			 pdata->wlc_en, pdata->wlc_en_act_low);
	}

	/* WLC_BPP_EPP_SLCT */
	pdata->slct_gpio = p9221_parse_gpios(dev, "idt,slct-gpio", "idt,gpio_slct", &flags);
	if (pdata->slct_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->slct_gpio > 0) {
		ret = of_property_read_u32(node, "idt,gpio_slct_value", &data);
		if (ret == 0)
			pdata->slct_value = (data != 0);
		dev_info(dev, "WLC_BPP_EPP_SLCT gpio:%d value=%d",
			 pdata->slct_gpio, pdata->slct_value);
	}

	/* QI_EXT_LDO_EN */
	pdata->ldo_en_gpio = p9221_parse_gpios(dev, "idt,ldo_en-gpio", "idt,gpio_ldo_en", &flags);
	if (pdata->ldo_en_gpio > 0)
		dev_info(dev, "QI_EXT_LDO_EN gpio:%d", pdata->ldo_en_gpio);

	pdata->wcin_inlim_en_gpio = p9221_parse_gpios(dev,
						      "google,wcin_inlim_en-gpio",
						      "google,wcin_inlim_en", &flags);
	if (pdata->wcin_inlim_en_gpio > 0)
		dev_info(dev, "WCIN_INLIM_EN gpio: %d", pdata->wcin_inlim_en_gpio);

	/* RTx: idt,gpio_ben / idt,gpio_switch / idt,gpio_boost */
	ret = of_property_read_u32(node, "idt,has_rtx", &data);
	if (ret == 0)
		pdata->has_rtx = !!data;
	else
		pdata->has_rtx =
		    ((pdata->chip_id == P9412_CHIP_ID) ||
		     (pdata->chip_id == P9382A_CHIP_ID));

	pdata->has_rtx_gpio = of_property_read_bool(node, "idt,has_rtx_gpio");

	dev_info(dev, "has_rtx:%d, has_rtx_gpio:%d\n", pdata->has_rtx, pdata->has_rtx_gpio);

	/* boost enable, power WLC IC from device */
	pdata->ben_gpio = p9221_parse_gpios(dev, "idt,gpio-ben", "idt,gpio_ben", &flags);
	if (pdata->ben_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->ben_gpio > 0)
		dev_info(dev, "ben gpio:%d\n", pdata->ben_gpio);

	pdata->switch_gpio = p9221_parse_gpios(dev, "idt,switch-gpio", "idt,gpio_switch", &flags);
	if (pdata->switch_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->switch_gpio > 0)
		dev_info(dev, "switch gpio:%d\n", pdata->switch_gpio);

	/* boost gpio sets rtx at charging voltage level */
	pdata->boost_gpio = p9221_parse_gpios(dev, "idt,boost-gpio", "idt,gpio_boost", &flags);
	if (pdata->boost_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->boost_gpio > 0)
		dev_info(dev, "boost gpio:%d\n", pdata->boost_gpio);

	/* configure boost to 7V through wlc chip */
	pdata->apbst_en = of_property_read_bool(node, "idt,apbst_en");
	/* support HW detect OCP in ping phase */
	pdata->hw_ocp_det = of_property_read_bool(node, "idt,hw_ocp_det");

	if (pdata->has_rtx)
		dev_info(dev, "RTx Config: ben:%d,switch:%d,boost:%d,apbst_en:%d,hw_ocp_det:%d\n",
			 pdata->ben_gpio, pdata->switch_gpio, pdata->boost_gpio,
			 pdata->apbst_en, pdata->hw_ocp_det);

	/* DC-PPS */
	pdata->ext_ben_gpio = p9221_parse_gpios(dev, "idt,extben-gpio", "idt,gpio_extben", &flags);
	if (pdata->ext_ben_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->ext_ben_gpio > 0) {
		ret = gpio_request(pdata->ext_ben_gpio, "wc_ref");
		dev_info(dev, "ext ben gpio:%d, ret=%d\n", pdata->ext_ben_gpio, ret);
	}

	pdata->dc_switch_gpio = p9221_parse_gpios(dev, "idt,dc_switch-gpio", "idt,gpio_dc_switch",
						  &flags);
	if (pdata->dc_switch_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->dc_switch_gpio > 0)
		dev_info(dev, "dc_switch gpio:%d\n", pdata->dc_switch_gpio);

	ret = of_property_read_u32(node, "idt,has_wlc_dc", &data);
	if (ret == 0)
		pdata->has_wlc_dc = !!data;
	else
		pdata->has_wlc_dc = pdata->chip_id == P9412_CHIP_ID;
	dev_info(dev, "has_wlc_dc:%d\n", pdata->has_wlc_dc);

	if (pdata->has_wlc_dc)
		dev_info(dev, "WLC-DC GPIO: ext_ben:%d,dc_switch:%d\n",
			 pdata->ext_ben_gpio, pdata->dc_switch_gpio);

	/* Main IRQ */
	pdata->irq_gpio = p9221_parse_gpios(dev, "idt,irq-gpio", "idt,irq_gpio", &flags);
	if (pdata->irq_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->irq_gpio > 0) {
		pdata->irq_int = gpio_to_irq(pdata->irq_gpio);
		ret = of_property_read_u32(node, "idt,irq_flag", &data);
		if (ret == 0)
			pdata->irq_flag = (u64)data;
		else
			pdata->irq_flag = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
		dev_info(dev, "gpio:%d, gpio_irq:%d irq_flag:0x%04llx\n",
			 pdata->irq_gpio, pdata->irq_int, pdata->irq_flag);
	}

	/* Optional Detect IRQ */
	pdata->irq_det_gpio = p9221_parse_gpios(dev, "idt,irq_det-gpio", "idt,irq_det_gpio",
						&flags);
	if (pdata->irq_det_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (pdata->irq_det_gpio > 0) {
		pdata->irq_det_int = gpio_to_irq(pdata->irq_det_gpio);
		dev_info(dev, "det gpio:%d, det gpio_irq:%d\n",
			 pdata->irq_det_gpio, pdata->irq_det_int);
	}

	/* Optional VOUT max */
	pdata->max_vout_mv = P9221_MAX_VOUT_SET_MV_DEFAULT;
	ret = of_property_read_u32(node, "idt,max_vout_mv", &data);
	if (ret == 0) {
		if (data < vout_set_min_mv || data > vout_set_max_mv)
			dev_err(dev, "max_vout_mv out of range %d\n", data);
		else
			pdata->max_vout_mv = data;
	}

	/* Optional FOD data */
	p9221_parse_fod(dev, &pdata->fod_num, pdata->fod, "fod");
	p9221_parse_fod(dev, &pdata->fod_lv_num, pdata->fod_lv, "fod_lv");
	p9221_parse_fod(dev, &pdata->fod_epp_num, pdata->fod_epp, "fod_epp");
	p9221_parse_fod(dev, &pdata->fod_gpp_num, pdata->fod_gpp, "fod_gpp");
	p9221_parse_fod(dev, &pdata->fod_epp_comp_num, pdata->fod_epp_comp, "fod_epp_comp");
	p9221_parse_fod(dev, &pdata->fod_epp_iop_num, pdata->fod_epp_iop, "fod_epp_iop");

	nb_hpp_fod_vol = of_property_count_elems_of_size(node, "google,hpp_fod_vol", sizeof(u32));
	if (nb_hpp_fod_vol > 0) {
		if (nb_hpp_fod_vol > P9412_HPP_FOD_SETS)
			nb_hpp_fod_vol = P9412_HPP_FOD_SETS;

		pdata->hpp_fod_vol = devm_kmalloc_array(dev, nb_hpp_fod_vol,
							sizeof(u32), GFP_KERNEL);
		if (pdata->hpp_fod_vol == NULL) {
			dev_warn(dev, "dt google,hpp_fod_vol array not created");
			return -ENOMEM;
		}
		ret = of_property_read_u32_array(node,
						 "google,hpp_fod_vol",
						 pdata->hpp_fod_vol,
						 nb_hpp_fod_vol);
		if (ret < 0) {
			dev_warn(dev, "failed to read google,hpp_fod_vol: %d\n", ret);
			pdata->hpp_fod_vol = NULL;
		} else {
			ret = p9221_parse_hpp_fods(dev, pdata, nb_hpp_fod_vol);
			if (ret == 0)
				pdata->nb_hpp_fod_vol = nb_hpp_fod_vol;
		}
	} else {
		nb_hpp_fod_vol = 2;
		pdata->hpp_fod_vol = devm_kmalloc_array(dev, nb_hpp_fod_vol,
							sizeof(u32), GFP_KERNEL);
		if (pdata->hpp_fod_vol == NULL) {
			dev_warn(dev, "dt google,hpp_fod_vol array not created");
			return -ENOMEM;
		}
		p9221_parse_fod(dev, &pdata->hpp_fods[0].num, pdata->hpp_fods[0].fod, "fod_hpp");
		p9221_parse_fod(dev, &pdata->hpp_fods[1].num, pdata->hpp_fods[1].fod, "fod_hpp_hv");
		if (pdata->hpp_fods[0].num > 0 && pdata->hpp_fods[1].num > 0) {
			pdata->hpp_fod_vol[0] = HPP_FOD_VOUT_THRESHOLD_UV;
			pdata->hpp_fod_vol[1] = pdata->max_vout_mv;
			pdata->nb_hpp_fod_vol = nb_hpp_fod_vol;
		} else {
			dev_warn(dev, "failed to read fod_hpp, fod_hpp_hv: %d\n", ret);
			pdata->hpp_fod_vol = NULL;
		}
	}

	ret = of_property_read_bool(node, "google,fod_fsw_base");
	if (ret)
		pdata->fod_fsw = true;

	ret = of_property_read_u32(node, "google,fod_fsw_high_thres", &data);
	if (ret < 0) {
		pdata->fod_fsw_high = -1;
	} else {
		pdata->fod_fsw_high = data;
		dev_info(dev, "dt fod_fsw_high_thres:%d\n", pdata->fod_fsw_high);
	}

	ret = of_property_read_u32(node, "google,fod_fsw_low_thres", &data);
	if (ret < 0) {
		pdata->fod_fsw_low = -1;
	} else {
		pdata->fod_fsw_low = data;
		dev_info(dev, "dt fod_fsw_low_thres:%d\n", pdata->fod_fsw_low);
	}

	pdata->fod_iop_mfg_num = of_property_count_elems_of_size(node, "google,iop_fod_mfg", sizeof(u16));
	if (pdata->fod_iop_mfg_num > 0) {
		if (pdata->fod_iop_mfg_num > P9XXX_IOP_MFG_NUM)
			pdata->fod_iop_mfg_num = P9XXX_IOP_MFG_NUM;
		ret = of_property_read_u16_array(node, "google,iop_fod_mfg",
						 pdata->fod_iop_mfg, pdata->fod_iop_mfg_num);
		if (ret == 0) {
			int i;

			for (i = 0; i < pdata->fod_iop_mfg_num; i++)
				dev_info(dev, "dt google,iop_fod_mfg: 0x%03x \n", pdata->fod_iop_mfg[i]);
		}
	}

	ret = of_property_read_u32(node, "google,q_value", &data);
	if (ret < 0) {
		pdata->q_value = -1;
	} else {
		pdata->q_value = data;
		dev_info(dev, "dt q_value:%d\n", pdata->q_value);
	}

	ret = of_property_read_u32(node, "google,rf_value", &data);
	if (ret < 0) {
		pdata->rf_value = -1;
	} else {
		pdata->rf_value = data;
		dev_info(dev, "dt rf_value:%d\n", pdata->rf_value);
	}

	ret = of_property_read_u32(node, "google,tx4191_q", &data);
	if (ret < 0) {
		pdata->tx_4191q = -1;
	} else {
		pdata->tx_4191q = data;
		dev_info(dev, "dt tx4191_q:%d\n", pdata->tx_4191q);
	}
	ret = of_property_read_u32(node, "google,tx1801_q", &data);
	if (ret < 0) {
		pdata->tx_1801q = -1;
	} else {
		pdata->tx_1801q = data;
		dev_info(dev, "dt tx1801_q:%d\n", pdata->tx_1801q);
	}
	ret = of_property_read_u32(node, "google,tx2356_q", &data);
	if (ret < 0) {
		pdata->tx_2356q = -1;
	} else {
		pdata->tx_2356q = data;
		dev_info(dev, "dt tx2356_q:%d\n", pdata->tx_2356q);
	}

	ret = of_property_read_u32(node, "google,epp_rp_value", &data);
	if (ret < 0) {
		pdata->epp_rp_value = -1;
	} else {
		pdata->epp_rp_value = data;
		dev_info(dev, "dt epp_rp_value: %d\n", pdata->epp_rp_value);
	}

	ret = of_property_read_u32(node, "google,epp_rp_low_value", &data);
	if (ret < 0) {
		pdata->epp_rp_low_value = -1;
	} else {
		pdata->epp_rp_low_value = data;
		dev_info(dev, "dt epp_rp_low_value: %d\n", pdata->epp_rp_low_value);
	}

	pdata->epp_vout_mv = P9221_MAX_VOUT_SET_MV_DEFAULT;
	ret = of_property_read_u32(node, "google,epp_vout_mv", &data);
	if (ret == 0) {
		if (data < vout_set_min_mv || data > vout_set_max_mv)
			dev_err(dev, "epp_vout_mv out of range %d\n", data);
		else
			pdata->epp_vout_mv = data;
	}

	ret = of_property_read_u32(node, "google,needs_dcin_reset", &data);
	if (ret < 0) {
		pdata->needs_dcin_reset = -1;
	} else {
		pdata->needs_dcin_reset = data;
		dev_info(dev, "dt needs_dcin_reset: %d\n",
			 pdata->needs_dcin_reset);
	}

	pdata->nb_alignment_freq =
			of_property_count_elems_of_size(node,
							"google,alignment_frequencies",
							sizeof(u32));
	dev_info(dev, "dt google,alignment_frequencies size = %d\n",
		 pdata->nb_alignment_freq);

	if (pdata->nb_alignment_freq > 0) {
		pdata->alignment_freq =
				devm_kmalloc_array(dev,
						   pdata->nb_alignment_freq,
						   sizeof(u32),
						   GFP_KERNEL);
		if (!pdata->alignment_freq) {
			dev_warn(dev,
				 "dt google,alignment_frequencies array not created");
		} else {
			ret = of_property_read_u32_array(node,
							 "google,alignment_frequencies",
							 pdata->alignment_freq,
							 pdata->nb_alignment_freq);
			if (ret) {
				dev_warn(dev,
					 "failed to read google,alignment_frequencies: %d\n",
					 ret);
				devm_kfree(dev, pdata->alignment_freq);
			}
		}
	}

	ret = of_property_read_u32(node, "google,alignment_scalar", &data);
	if (ret < 0)
		pdata->alignment_scalar = WLC_ALIGN_DEFAULT_SCALAR;
	else {
		pdata->alignment_scalar = data;
		if (pdata->alignment_scalar != WLC_ALIGN_DEFAULT_SCALAR)
			dev_info(dev, "google,alignment_scalar updated to: %d\n",
				 pdata->alignment_scalar);
	}

	ret = of_property_read_u32(node, "google,alignment_hysteresis", &data);
	if (ret < 0)
		pdata->alignment_hysteresis = WLC_ALIGN_DEFAULT_HYSTERESIS;
	else
		pdata->alignment_hysteresis = data;

	dev_info(dev, "google,alignment_hysteresis set to: %d\n",
				 pdata->alignment_hysteresis);

	ret = of_property_read_bool(node, "idt,ramp-disable");
	if (ret)
		pdata->icl_ramp_delay_ms = -1;

	ret = of_property_read_u32(node, "google,alignment_scalar_low_current",
				   &data);
	if (ret < 0)
		pdata->alignment_scalar_low_current = 0;
	else
		pdata->alignment_scalar_low_current = data;

	ret = of_property_read_u32(node, "google,alignment_scalar_high_current",
				   &data);
	if (ret < 0)
		pdata->alignment_scalar_high_current = 0;
	else
		pdata->alignment_scalar_high_current = data;

	ret = of_property_read_u32(node, "google,alignment_offset_low_current",
				   &data);
	if (ret < 0)
		pdata->alignment_offset_low_current = 0;
	else
		pdata->alignment_offset_low_current = data;

	ret = of_property_read_u32(node, "google,alignment_offset_high_current",
				   &data);
	if (ret < 0)
		pdata->alignment_offset_high_current = 0;
	else
		pdata->alignment_offset_high_current = data;

	ret = of_property_read_u32(node, "google,alignment_current_threshold",
				   &data);
	if (ret < 0)
		pdata->alignment_current_threshold = 0;
	else
		pdata->alignment_current_threshold = data;

	pdata->disable_align = !(pdata->alignment_scalar_low_current &&
				 pdata->alignment_scalar_high_current &&
				 pdata->alignment_offset_low_current &&
				 pdata->alignment_offset_high_current &&
				 pdata->alignment_current_threshold);
	dev_info(dev, "align:%s, scalar_low=%d, scalar_high=%d, "
		 "offset_low=%d, offset_high=%d, current_thres=%d\n",
		 pdata->disable_align ? "disable" : "enable", pdata->alignment_scalar_low_current,
		 pdata->alignment_scalar_high_current, pdata->alignment_offset_low_current,
		 pdata->alignment_offset_high_current, pdata->alignment_current_threshold);


	ret = of_property_read_u32(node, "google,power_mitigate_threshold",
				   &data);
	if (ret < 0)
		pdata->power_mitigate_threshold = 0;
	else
		pdata->power_mitigate_threshold = data;

	ret = of_property_read_bool(node, "google,feat-no-compat");
	if (!ret)
		pdata->feat_compat_mode = true; /* default is compat*/

	ret = of_property_read_bool(node, "google,has-sw-ramp");
	if (ret)
		pdata->has_sw_ramp = true;

	ret = of_property_read_u8(node,"idt,tx_id_phone_type",
				  &pdata->phone_type);
	if (ret < 0)
		pdata->phone_type = 0;

	ret = of_property_read_u32(node, "google,gpp_dcicl_default_ua", &data);
	if (ret < 0)
		pdata->dc_icl_gpp = 0;
	else
		pdata->dc_icl_gpp = data;

	ret = of_property_read_u32(node, "google,epp_dcicl_default_ma", &data);
	if (ret < 0)
		pdata->epp_icl = 0;
	else
		pdata->epp_icl = data;

	ret = of_property_read_u32(node, "google,gpp_dcicl_ua", &data);
	if (ret < 0)
		pdata->gpp_icl = P9XXX_DC_ICL_GPP_UA;
	else
		pdata->gpp_icl = data;

	ret = of_property_read_s32(node, "google,align_delta", &data);
	if (ret < 0)
		pdata->align_delta = 0;
	else
		pdata->align_delta = data;

	ret = of_property_read_u32(node, "google,bpp_iop_vout_mv", &data);
	if (ret < 0)
		pdata->set_iop_vout_bpp = 0;
	else
		pdata->set_iop_vout_bpp = data;

	ret = of_property_read_u32(node, "google,epp_iop_vout_mv", &data);
	if (ret < 0)
		pdata->set_iop_vout_epp = 0;
	else
		pdata->set_iop_vout_epp = data;

	ret = of_property_read_u32(node, "google,lowest-freq-limit-khz", &data);
	if (ret < 0)
		pdata->lowest_fsw_khz = 0;
	else
		pdata->lowest_fsw_khz = data;

	/* Calibrate light load */
	pdata->light_load = of_property_read_bool(node, "google,light_load");
	pdata->ll_vout_not_set = of_property_read_bool(node, "google,ll-bpp-vout-not-set");
	pdata->needs_align_check = of_property_read_bool(node, "google,align_check");

	pdata->disable_repeat_eop = of_property_read_bool(node, "google,disable-repeat-eop");

	ret = of_property_read_u32(node, "google,hpp_neg_pwr", &data);
	if (ret < 0)
		pdata->hpp_neg_pwr = HPP_MODE_PWR_REQUIRE;
	else
		pdata->hpp_neg_pwr = data;
	ret = of_property_read_u32(node, "google,epp_neg_pwr", &data);
	if (ret < 0)
		pdata->epp_neg_pwr = EPP_MODE_REQ_PWR;
	else
		pdata->epp_neg_pwr = data;
	ret = of_property_read_u32(node, "google,wait_prop_irq_ms", &data);
	if (ret < 0)
		pdata->wait_prop_irq_ms = WAIT_PROP_IRQ_MS;
	else
		pdata->wait_prop_irq_ms = data;

	ret = of_property_read_u32(node, "idt,gpp-cmfet", &data);
	pdata->gpp_cmfet = (ret == 0) ? data : 0;

	pdata->bpp_cep_on_dl = of_property_read_bool(node, "google,bpp-cep-on-dl");
	pdata->gpp_enhanced = of_property_read_bool(node, "google,gpp_enhanced");
	pdata->enable_15w = of_property_read_bool(node, "google,enable_15w");
	pdata->hda_tz_wlc = of_property_read_bool(node, "google,hda-tz-wlc");
	pdata->magsafe_optimized = of_property_read_bool(node, "google,magsafe-optimized");

	ret = of_property_read_u32(node, "google,bpp_dcicl_default_ua", &data);
	if (ret == 0)
		pdata->bpp_icl = data;

	ret = of_property_read_u32(node, "google,bpp_dcicl_lower_vout_ua", &data);
	if (ret == 0)
		pdata->bpp_lv_icl = data;

	ret = of_property_read_u32(node, "google,bpp_dcicl_ramp_ua", &data);
	if (ret == 0)
		pdata->bpp_icl_ramp_ua = data;

	ret = of_property_read_u32(node, "google,bpp_dcicl_lower_vout_ramp_ua", &data);
	if (ret == 0)
		pdata->bpp_lv_icl_ramp_ua = data;

	ret = of_property_read_u32(node, "google,tx2767_icl_ua", &data);
	if (ret == 0)
		pdata->tx_2767_icl = data;

	pdata->freq_108_disable_ramp = of_property_read_bool(node,
						"google,bpp-freq108-disable-ramp");

	ret = of_property_read_u8(node, "google,bpp_ask_mod_fet", &pdata->ask_mod_fet);
	if (ret < 0)
		pdata->ask_mod_fet = 0;

	ret = of_property_read_u32(node, "google,bpp_freq109_icl_ma", &data);
	if (ret == 0)
		pdata->freq_109_icl = data;

	ret = of_property_read_u32(node, "google,bpp_freq109_vout_mv", &data);
	if (ret == 0)
		pdata->freq_109_vout = data;

	return 0;
}

static enum power_supply_property p9221_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_QC_COMPAT
	POWER_SUPPLY_PROP_AICL_DELAY,
	POWER_SUPPLY_PROP_AICL_ICL,
#endif
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct gbms_desc p9221_psy_desc = {
	.psy_dsc.name = "wireless",
	.psy_dsc.type = POWER_SUPPLY_TYPE_WIRELESS,
	.psy_dsc.properties = p9221_props,
	.psy_dsc.num_properties = ARRAY_SIZE(p9221_props),
	.psy_dsc.get_property = p9221_get_property,
	.psy_dsc.set_property = p9221_set_property,
	.psy_dsc.property_is_writeable = p9221_prop_is_writeable,
	.get_property = p9221_gbms_get_property,
	.set_property = p9221_gbms_set_property,
	.property_is_writeable = p9221_gbms_prop_is_writeable,
	.psy_dsc.no_thermal = true,
	.forward = true,
};

static int p9382a_tx_icl_vote_callback(struct gvotable_election *el,
				       const char *reason, void *vote)
{
	struct p9221_charger_data *charger = gvotable_get_data(el);
	int icl_ua = GVOTABLE_PTR_TO_INT(vote);
	int ret = 0;

	if (icl_ua == 0 &&
	    (strcmp(reason, THERMAL_DAEMON_VOTER) == 0 ||
	    strcmp(reason, REASON_MDIS) == 0)) {
		charger->rtx_err |= RTX_OVER_TEMP_BIT;
		schedule_work(&charger->uevent_work);
		logbuffer_log(charger->rtx_log, "tx_icl: %d, voter: %s", icl_ua, reason);
	} else if (icl_ua && charger->rtx_err & RTX_OVER_TEMP_BIT) {
		charger->rtx_err &= ~RTX_OVER_TEMP_BIT;
		schedule_work(&charger->uevent_work);
	}

	if (!charger->ben_state)
		return 0;

	if (icl_ua == 0) {
		schedule_work(&charger->rtx_disable_work);
	} else {
		ret = charger->chip_set_tx_ilim(charger,
						P9221_UA_TO_MA(icl_ua));
		if (ret == 0)
			logbuffer_log(charger->rtx_log, "set TX_ICL to %dmA",
				      icl_ua);
		else
			dev_err(&charger->client->dev,
				"Couldn't set Tx current limit rc=%d\n", ret);
	}

	return 0;
}

/* called from */
int p9221_wlc_disable(struct p9221_charger_data *charger, int disable, u8 reason)
{
	int ret = 0;

	if ((disable && charger->online) || charger->send_eop) {
		int rc;

		ret = charger->chip_send_eop(charger, reason);

		rc = p9xxx_chip_set_cmfet_reg(charger, charger->wlc_disable_comcap);
		rc |= p9xxx_chip_set_hivout_cmfet_reg(charger, charger->wlc_disable_comcap);

		pr_info("Disabled Rx communication channel(CMFET): 0xF4 & 0x11B (%d)\n", rc);
		charger->send_eop = false;
	}

	/*
	 * could also change ->qien_gpio (e.g pull low when disable == 0)
	 * and/or toggle inhibit via ->qi_vbus_en.
	 * NOTE: using ->qien_gpio to disable the IC while VOUT sis present
	 * might (is) not supported.
	 */

	pr_debug("%s: disable=%d, ept_reason=%d ret=%d\n", __func__,
		 disable, disable ? reason : -1, ret);

	return ret;
}

static int p9221_wlc_disable_callback(struct gvotable_election *el,
				      const char *reason, void *vote)
{
	struct p9221_charger_data *charger = gvotable_get_data(el);
	int disable = GVOTABLE_PTR_TO_INT(vote);
	u8 val = P9221_EOP_UNKNOWN;

	if (!charger->online_spoof &&
	    disable && (charger->last_disable == 0 || charger->last_disable == -1) &&
	    charger->online && gvotable_get_int_vote(el, P9221_WLC_VOTER)) {
		logbuffer_prlog(charger->log, "wlc_disable: online_spoof=1");
		enable_irq(charger->pdata->irq_det_int);
		charger->online_spoof = true;
		cancel_delayed_work(&charger->stop_online_spoof_work);
	}

	if (charger->online_spoof && charger->last_disable && !disable)
		schedule_delayed_work(&charger->stop_online_spoof_work, msecs_to_jiffies(2000));

	if (charger->last_disable != disable)
		logbuffer_prlog(charger->log, "set wlc %s, vote=%s",
				disable ? "disable" : "enable", reason);
	charger->last_disable = disable;

	if (charger->pdata->wlc_en == charger->pdata->qien_gpio) {
		int value;
		value = (!disable) ^ charger->pdata->wlc_en_act_low;
		gpio_direction_output(charger->pdata->wlc_en, value);
		return 0;
	}

	charger->send_eop = gvotable_get_int_vote(charger->dc_icl_votable,
						  THERMAL_DAEMON_VOTER) == 0;
	if (!gvotable_get_int_vote(el, P9221_WLC_VOTER) && !charger->send_eop)
		val = P9221_EOP_RESTART_POWER; /* auto restart */

	p9221_wlc_disable(charger, disable, val);

	return 0;
}

static int fan_level_cb(struct gvotable_election *el,
			const char *reason, void *vote)
{
	struct p9221_charger_data *charger = gvotable_get_data(el);
	int lvl = GVOTABLE_PTR_TO_INT(vote);

	if (!charger)
		return 0;

	if (charger->fan_last_level == lvl)
		return 0;

	if (!charger->online)
		return 0;

	logbuffer_log(charger->log, "FAN_LEVEL %d->%d reason=%s",
		      charger->fan_last_level, lvl, reason ? reason : "<>");

	charger->fan_last_level = lvl;

	kobject_uevent(&charger->dev->kobj, KOBJ_CHANGE);

	return 0;
}

/*
 *  If able to read the chip_id register then we know we are online
 *
 *  Returns true when online.
 */
static bool p9221_check_online(struct p9221_charger_data *charger)
{
	int ret;
	u16 chip_id;

	/* Test to see if the charger is online */
	ret = p9221_reg_read_16(charger, P9221_CHIP_ID_REG, &chip_id);
	if (ret == 0) {
		dev_info(charger->dev, "Charger online id:%04x\n", chip_id);
		return true;
	}

	return false;
}

static void p9221_soc_work(struct work_struct *work)
{
	struct p9221_charger_data *charger = container_of(work,
			struct p9221_charger_data, soc_work.work);
	union power_supply_propval prop = { };
	int err, soc_raw;

	if (!charger->batt_psy) {
		static struct power_supply *psy[2];

		err = power_supply_get_by_phandle_array(charger->dev->of_node,
							"idt,fuel-gauge",
							psy, ARRAY_SIZE(psy));
		if (err < 0 || IS_ERR_OR_NULL(psy[0])) {
			schedule_delayed_work(&charger->soc_work, msecs_to_jiffies(1000));
			pr_info("%s: wait for fg err=%d\n", __func__, err);
			return;
		}

		dev_info(charger->dev, "Reading CSP from %s\n",
			 psy[0]->desc && psy[0]->desc->name ? psy[0]->desc->name : "<>");
		charger->batt_psy = psy[0];
	}

	/* triggered from notifier_cb */
	soc_raw = p9221_capacity_raw(charger);
	err = power_supply_get_property(charger->batt_psy, POWER_SUPPLY_PROP_STATUS, &prop);
	if (err == 0 && (prop.intval == POWER_SUPPLY_STATUS_FULL))
		soc_raw = 101;

	dev_dbg(charger->dev, "p9221_soc_work: soc=%d, err=%d\n", soc_raw, err);

	if (soc_raw >= 0)
		p9221_set_capacity(charger, soc_raw);
}

static bool p9xxx_find_votable(struct p9221_charger_data *charger)
{
	/*
	 * Find the DC_ICL votable, we use this to limit the current that
	 * is taken from the wireless charger.
	 */
	if (!charger->dc_icl_votable)
		charger->dc_icl_votable = gvotable_election_get_handle("DC_ICL");
	if (!charger->dc_icl_votable)
		dev_warn(&charger->client->dev, "Could not find DC_ICL votable\n");

	if (charger->pdata->ldo_en_gpio > 0 && !charger->wlc_spoof_votable)
		charger->wlc_spoof_votable = gvotable_election_get_handle("WLC_SPOOF");
	if (charger->pdata->ldo_en_gpio > 0 && !charger->wlc_spoof_votable)
		dev_warn(&charger->client->dev, "Could not find WLC SPOOF votable\n");
	/*
	 * Find the DC_SUSPEND, we use this to disable DCIN before
	 * enter RTx mode
	 */
	if (!charger->dc_suspend_votable)
		charger->dc_suspend_votable = gvotable_election_get_handle("DC_SUSPEND");
	if (!charger->dc_suspend_votable)
		dev_warn(&charger->client->dev, "Could not find DC_SUSPEND votable\n");

	if (!charger->chg_mode_votable)
		charger->chg_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (!charger->chg_mode_votable)
		dev_warn(&charger->client->dev, "Could not find %s votable\n", GBMS_MODE_VOTABLE);

	return charger->dc_icl_votable != NULL &&
	       charger->dc_suspend_votable != NULL &&
	       charger->chg_mode_votable != NULL &&
	       (charger->pdata->ldo_en_gpio > 0 && charger->wlc_spoof_votable != NULL);
}

static int p9221_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device_node *dn, *of_node = client->dev.of_node;
	struct p9221_charger_data *charger;
	struct p9221_charger_platform_data *pdata = client->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	bool online;
	int ret;

	ret = i2c_check_functionality(client->adapter,
				      I2C_FUNC_SMBUS_BYTE_DATA |
				      I2C_FUNC_SMBUS_WORD_DATA |
				      I2C_FUNC_SMBUS_I2C_BLOCK);
	if (!ret) {
		ret = i2c_get_functionality(client->adapter);
		dev_err(&client->dev, "I2C adapter not compatible %x\n", ret);
		return -ENOSYS;
	}

	if (of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate pdata\n");
			return -ENOMEM;
		}

		ret = p9221_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_warn(&client->dev, "pdata is not ready\n");
			return ret;
		}
	}

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (charger == NULL) {
		dev_err(&client->dev, "Failed to allocate charger\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, charger);
	charger->dev = &client->dev;
	charger->dev->init_name = "i2c-p9221";
	charger->client = client;
	charger->pdata = pdata;
	charger->resume_complete = true;
	charger->align = WLC_ALIGN_ERROR;
	charger->align_count = 0;
	charger->is_mfg_google = false;
	charger->fw_rev = 0;
	charger->p9412_gpio_ctl = false;
	charger->chip_id = charger->pdata->chip_id;
	charger->rtx_wakelock = false;
	charger->last_disable = -1;
	charger->irq_at = 0;
	charger->check_rp = RP_NOTSET;
	charger->extended_int_recv = false;
	charger->trigger_dd = DREAM_DEBOUNCE_TIME_S;
	charger->det_status = 1;
	charger->set_auth_icl = false;
	charger->fod_mode = -1;
	mutex_init(&charger->io_lock);
	mutex_init(&charger->cmd_lock);
	mutex_init(&charger->stats_lock);
	mutex_init(&charger->chg_features.feat_lock);
	mutex_init(&charger->rtx_lock);
	mutex_init(&charger->rtx_gpio_lock);
	mutex_init(&charger->auth_lock);
	mutex_init(&charger->renego_lock);
	mutex_init(&charger->fod_lock);
	mutex_init(&charger->irq_det_lock);
	mutex_init(&charger->icl_lock);
	timer_setup(&charger->vrect_timer, p9221_vrect_timer_handler, 0);
	timer_setup(&charger->align_timer, p9221_align_timer_handler, 0);
	INIT_DELAYED_WORK(&charger->dcin_work, p9221_dcin_work);
	INIT_DELAYED_WORK(&charger->stop_online_spoof_work, p9xxx_stop_online_spoof_work);
	INIT_DELAYED_WORK(&charger->change_det_status_work, p9xxx_change_det_status_work);
	INIT_DELAYED_WORK(&charger->charge_stats_hda_work, p9221_charge_stats_hda_work);
	INIT_DELAYED_WORK(&charger->tx_work, p9221_tx_work);
	INIT_DELAYED_WORK(&charger->txid_work, p9382_txid_work);
	INIT_DELAYED_WORK(&charger->icl_ramp_work, p9221_icl_ramp_work);
	INIT_DELAYED_WORK(&charger->align_work, p9221_align_work);
	INIT_DELAYED_WORK(&charger->dcin_pon_work, p9221_dcin_pon_work);
	INIT_DELAYED_WORK(&charger->rtx_work, p9382_rtx_work);
	INIT_DELAYED_WORK(&charger->auth_dc_icl_work, p9221_auth_dc_icl_work);
	INIT_DELAYED_WORK(&charger->soc_work, p9221_soc_work);
	INIT_DELAYED_WORK(&charger->chk_rp_work, p9xxx_chk_rp_work);
	INIT_DELAYED_WORK(&charger->chk_rtx_ocp_work, p9412_chk_rtx_ocp_work);
	INIT_DELAYED_WORK(&charger->chk_fod_work, p9xxx_chk_fod_work);
	INIT_DELAYED_WORK(&charger->set_rf_work, p9xxx_set_rf_work);
	INIT_WORK(&charger->uevent_work, p9221_uevent_work);
	INIT_WORK(&charger->calibration_work, p9xxx_calibration_work);
	INIT_WORK(&charger->rtx_disable_work, p9382_rtx_disable_work);
	INIT_WORK(&charger->rtx_reset_work, p9xxx_rtx_reset_work);
	INIT_DELAYED_WORK(&charger->power_mitigation_work,
			  p9221_power_mitigation_work);
	alarm_init(&charger->icl_ramp_alarm, ALARM_BOOTTIME,
		   p9221_icl_ramp_alarm_cb);
	alarm_init(&charger->auth_dc_icl_alarm, ALARM_BOOTTIME,
		   p9221_auth_dc_icl_alarm_cb);

	init_waitqueue_head(&charger->ccreset_wq);

	charger->align_ws = wakeup_source_register(NULL, "p9221_align");
	charger->det_status_ws = wakeup_source_register(NULL, "p9221_det_status");

	/* setup function pointers for platform */
	/* first from *_charger.c -> *_chip.c */
	charger->reg_read_n = p9221_reg_read_n;
	charger->reg_read_8 = p9221_reg_read_8;
	charger->reg_read_16 = p9221_reg_read_16;
	charger->reg_write_n = p9221_reg_write_n;
	charger->reg_write_8 = p9221_reg_write_8;
	charger->reg_write_16 = p9221_reg_write_16;
	/* then from *_chip.c -> *_charger.c */
	p9221_chip_init_params(charger, charger->pdata->chip_id);
	p9221_chip_init_interrupt_bits(charger, charger->pdata->chip_id);
	ret = p9221_chip_init_funcs(charger, charger->pdata->chip_id);
	if (ret) {
		dev_err(&client->dev,
			"Failed to initialize chip specific information\n");
		return ret;
	}

	p9221_charge_stats_init(&charger->chg_data);

	if (charger->pdata->qien_gpio > 0)
		gpio_direction_output(charger->pdata->qien_gpio, 0);

	if (charger->pdata->ldo_en_gpio > 0)
		gpio_direction_output(charger->pdata->ldo_en_gpio, 0);

	if (charger->pdata->qi_vbus_en > 0)
		gpio_direction_output(charger->pdata->qi_vbus_en,
				      !charger->pdata->qi_vbus_en_act_low);

	if (charger->pdata->slct_gpio > 0)
		gpio_direction_output(charger->pdata->slct_gpio, charger->pdata->slct_value);

	if (charger->pdata->wcin_inlim_en_gpio > 0)
		gpio_direction_output(charger->pdata->wcin_inlim_en_gpio, 0);

	if (charger->pdata->ben_gpio > 0)
		gpio_direction_output(charger->pdata->ben_gpio, 0);

	if (charger->pdata->switch_gpio > 0)
		gpio_direction_output(charger->pdata->switch_gpio, 0);

	if (charger->pdata->ext_ben_gpio > 0)
		gpio_direction_output(charger->pdata->ext_ben_gpio, 0);

	if (charger->pdata->dc_switch_gpio > 0)
		gpio_direction_output(charger->pdata->dc_switch_gpio, 0);

	/* Default enable */
	charger->enabled = true;

	/* Default to R5+ */
	charger->cust_id = 5;

	psy_cfg.drv_data = charger;
	psy_cfg.of_node = charger->dev->of_node;
	charger->wc_psy = devm_power_supply_register(charger->dev,
						     &p9221_psy_desc.psy_dsc,
						     &psy_cfg);
	if (IS_ERR(charger->wc_psy)) {
		dev_err(&client->dev, "Fail to register supply: %d\n", ret);
		return PTR_ERR(charger->wc_psy);
	}

	/*
	 * Create the WLC_DISABLE votable, use for send EPT
	 * NOTE: pulling QI_EN_L might not be OK, verify this with EE
	 */
	charger->wlc_disable_votable =
		gvotable_create_bool_election(NULL, p9221_wlc_disable_callback,
					      charger);
	if (IS_ERR(charger->wlc_disable_votable)) {
		ret = PTR_ERR(charger->wlc_disable_votable);
		dev_err(&client->dev,
			"Couldn't create WLC_DISABLE rc=%d\n", ret);
		charger->wlc_disable_votable = NULL;
	} else {
		gvotable_set_vote2str(charger->wlc_disable_votable,
				      gvotable_v2s_int);
		gvotable_election_set_name(charger->wlc_disable_votable,
					   "WLC_DISABLE");
	}

	/*
	 * Create the RTX_ICL votable, we use this to limit the current that
	 * is taken for RTx mode
	 */
	if (charger->pdata->has_rtx) {
		charger->tx_icl_votable =
			gvotable_create_int_election(
				NULL, gvotable_comparator_int_min,
				p9382a_tx_icl_vote_callback, charger);
		if (IS_ERR(charger->tx_icl_votable)) {
			ret = PTR_ERR(charger->tx_icl_votable);
			dev_err(&client->dev,
				"Couldn't create TX_ICL rc=%d\n", ret);
			charger->tx_icl_votable = NULL;
		} else {
			gvotable_set_vote2str(charger->tx_icl_votable,
					      gvotable_v2s_int);
			gvotable_election_set_name(charger->tx_icl_votable,
						   "TX_ICL");
			/* vote default TX_ICL for rtx mode */
			gvotable_cast_long_vote(
				charger->tx_icl_votable, P9382A_RTX_VOTER,
				P9221_MA_TO_UA(P9382A_RTX_ICL_MAX_MA), true);
		}
	}

	charger->fan_level_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_max,
					     fan_level_cb, charger);
	if (IS_ERR_OR_NULL(charger->fan_level_votable)) {
		ret = PTR_ERR(charger->fan_level_votable);
		dev_err(&client->dev, "Fail to create fan_level_votable\n");
		charger->fan_level_votable = NULL;
	} else {
		gvotable_set_vote2str(charger->fan_level_votable,
				      gvotable_v2s_int);
		gvotable_election_set_name(charger->fan_level_votable,
					   VOTABLE_FAN_LEVEL);
		gvotable_cast_long_vote(charger->fan_level_votable,
					"DEFAULT", FAN_LVL_UNKNOWN, true);
	}

	charger->fan_last_level = -1;

	charger->votable_init_done = p9xxx_find_votable(charger);

	/* Ramping on BPP is optional */
	if (charger->pdata->icl_ramp_delay_ms != -1) {
		charger->icl_ramp_ua = P9221_DC_ICL_BPP_RAMP_DEFAULT_UA;
		charger->pdata->icl_ramp_delay_ms =
					P9221_DC_ICL_BPP_RAMP_DELAY_DEFAULT_MS;
	}

	charger->dc_icl_bpp = 0;
	charger->dc_icl_epp = 0;
	charger->dc_icl_epp_neg = charger->pdata->epp_icl > 0 ? charger->pdata->epp_icl : P9221_DC_ICL_EPP_UA;
	charger->aicl_icl_ua = 0;
	charger->aicl_delay_ms = 0;

	crc8_populate_msb(p9221_crc8_table, P9221_CRC8_POLYNOMIAL);

	online = p9221_check_online(charger);
	dev_info(&client->dev, "online = %d CHIP_ID = 0x%x\n", online,
		 charger->chip_id);

	if (online) {
		/* set charger->online=true, will ignore first VRECTON IRQ */
		p9221_set_online(charger);
		if (charger->alignment == -1)
			p9221_init_align(charger);
	} else {
		/* disconnected, (likely err!=0) vote for BPP */
		p9221_vote_defaults(charger);
	}

	/*
	 * irq_type should use "IRQF_TRIGGER_LOW" or we will
	 * missing some interrupts.
	 * this issue happened on R3 b/154789273.
	 */
	if (charger->pdata->irq_int > 0) {
		ret = devm_request_threaded_irq(
				&client->dev, charger->pdata->irq_int, NULL,
				p9221_irq_thread, charger->pdata->irq_flag,
				"p9221-irq", charger);
		if (ret)
			dev_err(&client->dev, "Failed to request IRQ\n");
	}

	device_init_wakeup(charger->dev, true);
	ret = enable_irq_wake(charger->pdata->irq_int);
	if (ret)
		dev_err(&client->dev, "Error enabling irq wake ret:%d\n", ret);

	/*
	 * We will receive a VRECTON after enabling IRQ if the device is
	 * if the device is already in-field when the driver is probed.
	 */
	device_init_wakeup(charger->dev, true);

	if (gpio_is_valid(charger->pdata->irq_det_gpio)) {
		ret = devm_request_threaded_irq(&client->dev, charger->pdata->irq_det_int, NULL,
					p9221_irq_det_thread,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"p9221-irq-det", charger);
		if (ret) {
			dev_err(&client->dev, "Failed to request IRQ_DET\n");
		} else {
			mutex_lock(&charger->irq_det_lock);
			charger->det_status = gpio_get_value_cansleep(charger->pdata->irq_det_gpio);
			disable_irq(charger->pdata->irq_det_int);
			mutex_unlock(&charger->irq_det_lock);
		}
	}

	charger->last_capacity = -1;
	charger->count = 1;
	bin_attr_rxdata.size = charger->rx_buf_size;
	bin_attr_txdata.size = charger->tx_buf_size;
	ret = sysfs_create_group(&charger->dev->kobj, &p9221_attr_group);
	if (ret) {
		dev_info(&client->dev, "sysfs_create_group failed\n");
	}
	if (charger->pdata->has_rtx) {
		ret = sysfs_create_group(&charger->dev->kobj, &rtx_attr_group);
		if (ret)
			dev_info(&client->dev, "rtx sysfs_create_group failed\n");
	}

	charger->debug_entry = debugfs_create_dir("p9221_charger", 0);
	if (IS_ERR_OR_NULL(charger->debug_entry)) {
		charger->debug_entry = NULL;
		dev_err(&client->dev, "Failed to create debug_entry\n");
	} else {
		debugfs_create_u32("trigger_dd", 0644, charger->debug_entry, &charger->trigger_dd);
		debugfs_create_bool("no_fod", 0644, charger->debug_entry, &charger->no_fod);
		debugfs_create_bool("de_rtx_hw_ocp", 0644, charger->debug_entry,
				    &charger->pdata->hw_ocp_det);
		debugfs_create_u32("de_q_value", 0644, charger->debug_entry, &charger->de_q_value);
		debugfs_create_u32("de_chk_ocp_ms", 0644, charger->debug_entry,
				   &charger->rtx_ocp_chk_ms);
		debugfs_create_u32("de_rtx_delay_ms", 0644, charger->debug_entry,
				   &charger->rtx_total_delay);
		debugfs_create_bool("needs_align_check", 0644, charger->debug_entry,
				    &charger->pdata->needs_align_check);
		debugfs_create_file("irq_det", 0444, charger->debug_entry, charger,
				    &debug_irq_det_fops);
		debugfs_create_u32("det_on_debounce", 0644, charger->debug_entry,
				   &charger->det_on_debounce);
		debugfs_create_u32("det_off_debounce", 0644, charger->debug_entry,
				   &charger->det_off_debounce);
		debugfs_create_u32("de_hpp_neg_pwr", 0644, charger->debug_entry,
				   &charger->de_hpp_neg_pwr);
		debugfs_create_u32("de_epp_neg_pwr", 0644, charger->debug_entry,
				   &charger->de_epp_neg_pwr);
		debugfs_create_u32("de_wait_prop_irq_ms", 0644, charger->debug_entry,
				   &charger->de_wait_prop_irq_ms);
		debugfs_create_u16("de_rtx_ocp_ma", 0644, charger->debug_entry,
				   &charger->rtx_ocp);
		debugfs_create_u16("de_rtx_api_limit_ma", 0644, charger->debug_entry,
				   &charger->rtx_api_limit);
		debugfs_create_u16("de_rtx_fb_freq_low_khz", 0644, charger->debug_entry,
				   &charger->rtx_fb_freq_low_limit);
		debugfs_create_u16("de_rtx_hb_freq_low_khz", 0644, charger->debug_entry,
				   &charger->rtx_hb_freq_low_limit);
		debugfs_create_u16("de_rtx_fod_thrsh_mw", 0644, charger->debug_entry,
				   &charger->rtx_fod_thrsh);
		debugfs_create_u16("de_rtx_plim_ma", 0644, charger->debug_entry,
				   &charger->ra9530_rtx_plim);
		debugfs_create_u32("de_ocp_ua", 0644, charger->debug_entry,
				   &charger->wlc_ocp);
		debugfs_create_u32("enable_i2c_debug", 0644, charger->debug_entry,
				   &charger->enable_i2c_debug);
	}

	/* can independently read battery capacity */
	dn = of_parse_phandle(of_node, "idt,fuel-gauge", 0);
	if (dn)
		schedule_delayed_work(&charger->soc_work, 0);

	/*
	 * Register notifier so we can detect changes on DC_IN
	 */
	INIT_DELAYED_WORK(&charger->notifier_work, p9221_notifier_work);
	charger->nb.notifier_call = p9221_notifier_cb;
	ret = power_supply_reg_notifier(&charger->nb);
	if (ret) {
		dev_err(&client->dev, "Fail to register notifier: %d\n", ret);
		return ret;
	}

	charger->log = logbuffer_register("wireless");
	if (IS_ERR(charger->log)) {
		ret = PTR_ERR(charger->log);
		dev_err(charger->dev,
			"failed to obtain logbuffer instance, ret=%d\n", ret);
		charger->log = NULL;
	}

	charger->rtx_log = logbuffer_register("rtx");
	if (IS_ERR(charger->rtx_log)) {
		ret = PTR_ERR(charger->rtx_log);
		dev_err(charger->dev,
			"failed to obtain rtx logbuffer instance, ret=%d\n",
			ret);
		charger->rtx_log = NULL;
	}

#if IS_ENABLED(CONFIG_GPIOLIB)
	if (charger->pdata->chip_id == P9412_CHIP_ID ||
	    charger->pdata->chip_id == P9222_CHIP_ID ||
	    charger->pdata->chip_id == RA9530_CHIP_ID) {
		p9xxx_gpio_init(charger);
		charger->gpio.parent = &client->dev;
		charger->gpio.of_node = of_find_node_by_name(client->dev.of_node,
						charger->gpio.label);
		if (!charger->gpio.of_node)
			dev_err(&client->dev, "Failed to find %s DT node\n",
				charger->gpio.label);

		ret = devm_gpiochip_add_data(&client->dev, &charger->gpio, charger);
		dev_info(&client->dev, "%d GPIOs registered ret:%d\n",
			 charger->gpio.ngpio, ret);

	}
#endif

	dev_info(&client->dev, "p9221 Charger Driver Loaded\n");

	if (online) {
		charger->dc_psy = power_supply_get_by_name("dc");
		if (charger->dc_psy)
			power_supply_changed(charger->dc_psy);
	}

	/* prefill txid 1 with API revision number (1) */
	feature_update_cache(&charger->chg_features, 1, 1);
	return 0;
}

static void p9221_charger_remove(struct i2c_client *client)
{
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	power_supply_unreg_notifier(&charger->nb);
	cancel_delayed_work_sync(&charger->dcin_work);
	cancel_delayed_work_sync(&charger->stop_online_spoof_work);
	cancel_delayed_work_sync(&charger->change_det_status_work);
	cancel_delayed_work_sync(&charger->charge_stats_hda_work);
	cancel_delayed_work_sync(&charger->tx_work);
	cancel_delayed_work_sync(&charger->txid_work);
	cancel_delayed_work_sync(&charger->icl_ramp_work);
	cancel_delayed_work_sync(&charger->dcin_pon_work);
	cancel_delayed_work_sync(&charger->align_work);
	cancel_delayed_work_sync(&charger->rtx_work);
	cancel_delayed_work_sync(&charger->auth_dc_icl_work);
	cancel_delayed_work_sync(&charger->chk_rp_work);
	cancel_delayed_work_sync(&charger->chk_rtx_ocp_work);
	cancel_delayed_work_sync(&charger->chk_fod_work);
	cancel_delayed_work_sync(&charger->set_rf_work);
	cancel_work_sync(&charger->uevent_work);
	cancel_work_sync(&charger->calibration_work);
	cancel_work_sync(&charger->rtx_disable_work);
	cancel_work_sync(&charger->rtx_reset_work);
	cancel_delayed_work_sync(&charger->power_mitigation_work);
	alarm_try_to_cancel(&charger->icl_ramp_alarm);
	alarm_try_to_cancel(&charger->auth_dc_icl_alarm);
	del_timer_sync(&charger->vrect_timer);
	del_timer_sync(&charger->align_timer);
	disable_irq_wake(charger->pdata->irq_int);
	device_init_wakeup(charger->dev, false);
	cancel_delayed_work_sync(&charger->notifier_work);
	if (!IS_ERR_OR_NULL(charger->batt_psy))
		power_supply_put(charger->batt_psy);
	mutex_destroy(&charger->io_lock);
	mutex_destroy(&charger->stats_lock);
	mutex_destroy(&charger->chg_features.feat_lock);
	mutex_destroy(&charger->rtx_lock);
	mutex_destroy(&charger->rtx_gpio_lock);
	mutex_destroy(&charger->auth_lock);
	mutex_destroy(&charger->renego_lock);
	if (charger->log)
		logbuffer_unregister(charger->log);
	if (charger->rtx_log)
		logbuffer_unregister(charger->rtx_log);

	wakeup_source_unregister(charger->align_ws);
	wakeup_source_unregister(charger->det_status_ws);
	if (charger->i2c_rxdebug_buf)
		kfree(charger->i2c_rxdebug_buf);
	if (charger->i2c_txdebug_buf)
		kfree(charger->i2c_txdebug_buf);
}

static void p9221_charger_shutdown(struct i2c_client *client)
{
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	if (!charger)
		return;

	if (charger->online) {
		gpio_direction_output(charger->pdata->wlc_en, charger->pdata->wlc_en_act_low);
		dev_info(&charger->client->dev, "Disable WLC chip, wlc_en=%d, val=%d\n",
			 charger->pdata->wlc_en, charger->pdata->wlc_en_act_low);
	}
	power_supply_unreg_notifier(&charger->nb);
}

static const struct i2c_device_id p9221_charger_id_table[] = {
	{ "p9221", 0 },
	{ "p9382", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, p9221_charger_id_table);

#ifdef CONFIG_OF
static struct of_device_id p9221_charger_match_table[] = {
	{ .compatible = "idt,p9221",},
	{ .compatible = "idt,p9222",},
	{ .compatible = "idt,p9382",},
	{ .compatible = "idt,p9412",},
	{ .compatible = "idt,ra9530",},
	{},
};
#else
#define p9221_charger_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int p9221_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	dev_dbg(dev, "%s\n", __func__);

	charger->resume_complete = false;
	pm_runtime_put_sync(charger->dev);

	return 0;
}

static int p9221_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct p9221_charger_data *charger = i2c_get_clientdata(client);

	pm_runtime_get_sync(charger->dev);
	dev_dbg(dev, "%s\n", __func__);

	charger->resume_complete = true;
	pm_runtime_put_sync(charger->dev);

	return 0;
}
#endif
static const struct dev_pm_ops p9221_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(p9221_pm_suspend, p9221_pm_resume)
};

static struct i2c_driver p9221_charger_driver = {
	.driver = {
		.name		= "p9221",
		.owner		= THIS_MODULE,
		.of_match_table = p9221_charger_match_table,
		.pm		= &p9221_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= p9221_charger_probe,
	.remove		= p9221_charger_remove,
	.shutdown	= p9221_charger_shutdown,
	.id_table	= p9221_charger_id_table,
};
module_i2c_driver(p9221_charger_driver);
MODULE_DESCRIPTION("IDT P9221 Wireless Power Receiver Driver");
MODULE_AUTHOR("Patrick Tjin <pattjin@google.com>");
MODULE_LICENSE("GPL");
