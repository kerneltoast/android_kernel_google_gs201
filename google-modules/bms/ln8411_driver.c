// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for LN8411 Direct charger
 * Based on existing PCA9468 driver
 */


#include <linux/err.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#include "ln8411_regs.h"
#include "ln8411_charger.h"

#if IS_ENABLED(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#define POLL_ADC

/* Timer definition */
#define LN8411_VBATMIN_CHECK_T	1000	/* 1000ms */
#define LN8411_CCMODE_CHECK1_T	5000	/* 10000ms -> 500ms */
#define LN8411_CCMODE_CHECK2_T	5000	/* 5000ms */
#define LN8411_CVMODE_CHECK_T 	10000	/* 10000ms */
#define LN8411_ENABLE_DELAY_T	250	/* 250ms */
#define LN8411_CVMODE_CHECK2_T	1000	/* 1000ms */
#define LN8411_ENABLE_WLC_DELAY_T	300	/* 300ms */

/* Battery Threshold */
#define LN8411_DC_VBAT_MIN		3000000 /* uV */
/* Input Current Limit default value */
#define LN8411_IIN_CFG_DFT		2500000 /* uA*/
/* Charging Float Voltage default value */
#define LN8411_VFLOAT_DFT		4350000	/* uV */
/* Charging Float Voltage max voltage for comp */
#define LN8411_COMP_VFLOAT_MAX		4700000	/* uV */

/* Charging Done Condition */
#define LN8411_IIN_DONE_DFT		500000		/* uA */

/* Maximum TA voltage threshold */
#define LN8411_TA_MAX_VOL		10500000 /* uV */
#define LN8411_TA_MAX_VOL_2_1		11000000 /* UV */
/* Maximum TA current threshold, set to max(cc_max) / 2 */
#define LN8411_TA_MAX_CUR		2600000	 /* uA */
#define LN8411_TA_MAX_CUR_4_1		2250000	 /* uA */
/* Minimum TA current threshold */
#define LN8411_TA_MIN_CUR		1000000	/* uA - PPS minimum current */

/* Minimum TA voltage threshold in Preset mode */
#define LN8411_TA_MIN_VOL_PRESET	8000000	/* uV */

#define LN8411_TA_VOL_PRE_OFFSET	500000	 /* uV */
#define LN8411_WLC_VOL_PRE_OFFSET	500000   /* uV */
/* Adjust CC mode TA voltage step */
#define LN8411_TA_VOL_STEP_ADJ_CC	40000	/* uV */
/* Pre CV mode TA voltage step */
#define LN8411_TA_VOL_STEP_PRE_CV	20000	/* uV */

/* IIN_CC adc offset for accuracy */
#define LN8411_IIN_ADC_OFFSET		20000	/* uA */
/* IIN_CC compensation offset */
#define LN8411_IIN_CC_COMP_OFFSET	25000	/* uA */
/* IIN_CC compensation offset in Power Limit Mode(Constant Power) TA */
#define LN8411_IIN_CC_COMP_OFFSET_CP	20000	/* uA */
/* TA maximum voltage that can support CC in Constant Power Mode */
#define LN8411_TA_MAX_VOL_CP		10250000
/* Offset for cc_max / 2 */
#define LN8411_IIN_MAX_OFFSET		25000 /* uA */
/* Offset for TA max current */
#define LN8411_TA_CUR_MAX_OFFSET	200000 /* uA */

/* maximum retry counter for restarting charging */
#define LN8411_MAX_RETRY_CNT			3	/* retries */
#define LN8411_MAX_IBUS_UCP_RETRY_CNT		10	/* retries */
#define LN8411_MAX_IBUS_UCP_DEBOUNCE_COUNT	3
#define LN8411_MAX_LOW_BATT_RETRY_CNT		10	/* retries */

/* TA IIN tolerance */
#define LN8411_TA_IIN_OFFSET		100000	/* uA */

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP		20000	/* uV */
#define PD_MSG_TA_CUR_STEP		50000	/* uA */

/* Maximum WCRX voltage threshold */
#define LN8411_WCRX_MAX_VOL		9750000 /* uV */
/* WCRX voltage Step */
#define WCRX_VOL_STEP			40000	/* uV */

/* Default value for protections */
#define VBAT_REV_UVP_DFT		3500000 /* 3.5V */
#define VBAT_UVP_DFT			3400000 /* 3.4V */
#define VBAT_OVP_WARN_DFT		4450000 /* 4.45V */
#define VBAT_OVP_DFT			4525000 /* 4.525V */
#define VBAT_OVP_DFT_1_2		8 /* 4.650V */
#define SWCAP_OVP_DFT			750000 /* 0.75V */
#define SWCAP_UVP_DFT			0 /* 0V */
#define CBAT_OCP_WARN_DFT		8000000 /* 8A */
#define CBAT_OCP_DFT			8200000 /* 8.2A */
#define CBUS_OCP_WARN_DFT		2500000 /* 2.5A */
#define IBUS_OCP_DFT_4_1		0xE /* 2.3A */
#define IBUS_OCP_DFT_2_1		0x4 /* 3.5A */
#define IBUS_OCP_DFT_1_2		0x5 /* 1.4A */
#define VUSB_OVP_DFT_4_1		0x8b /* 22V */
#define VUSB_OVP_DFT_2_1		0x81 /* 12V */
#define VUSB_OVP_DFT_4_1_B0		0xab /* 12V */
#define VUSB_OVP_DFT_2_1_B0		0xa1 /* 12V */
#define IBUS_UCP_DFT_4_1		0x0
#define IBUS_UCP_DFT_2_1		0x3
#define IBUS_UCP_DFT_1_2		0x90
#define IBUS_UCP_DFT_1_2_B0             0x93
#define PMID2OUT_UVP_DFT_4_1		0x4 /* 10% */
#define PMID2OUT_UVP_DFT_1_2		0x84 /* Disable */
#define PMID2OUT_UVP_DFT_EN_1_2		0x4
#define CFG_10_DFT_4_1			0x1
#define CFG_10_DFT_2_1			0x0
#define PMID2OUT_UVP			0x84 /* 10% */
#define PMID_SWITCH_OK_DIS		0x78
#define LN8411_INFET_OFF_DET_DIS	0xd
#define VBAT_ALARM_CFG_DELTA		50000 /* 50mV */
#define IBUS_ALARM_CFG_DELTA		200000 /* 200mA */

#define ADC_EN_RETRIES			400

#define LN8411_TIER_SWITCH_DELTA		25000 /* uV */

/* GPIO support for 1_2 mode */
#define LN8411_NUM_GPIOS	1
#define LN8411_MIN_GPIO		0
#define LN8411_MAX_GPIO		0
#define LN8411_GPIO_1_2_EN	0

/* Status */
enum sts_mode_t {
	STS_MODE_CHG_LOOP,	/* TODO: There is no such thing */
	STS_MODE_VFLT_LOOP,
	STS_MODE_IIN_LOOP,
	STS_MODE_LOOP_INACTIVE,
	STS_MODE_CHG_DONE,
	STS_MODE_VIN_UVLO,
};

/* Timer ID */
enum timer_id_t {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_PRESET_DC,
	TIMER_PRESET_CONFIG,
	TIMER_CHECK_ACTIVE,
	TIMER_ADJUST_CCMODE,
	TIMER_CHECK_CCMODE,
	TIMER_ENTER_CVMODE,
	TIMER_CHECK_CVMODE, /* 8 */
	TIMER_PDMSG_SEND,   /* 9 */
	TIMER_ADJUST_TAVOL,
	TIMER_ADJUST_TACUR,
};


/* TA increment Type */
enum ta_inc_t {
	INC_NONE,	/* No increment */
	INC_TA_VOL,	/* TA voltage increment */
	INC_TA_CUR,	/* TA current increment */
};

/* BATT info Type */
enum batt_info_t {
	BATT_CURRENT,
	BATT_VOLTAGE,
};

/* Reg, val, mask (if update) */
static u8 mode_settings_A1[3][9][3] = {
	/* 2 to 1 */
	{
		{LN8411_VUSB_OVP, VUSB_OVP_DFT_2_1, 0xcf},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_2_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_2_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_2_1, 0x0},
		{LN8411_PMID2OUT_UVP, PMID2OUT_UVP_DFT_4_1, 0x9f},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
	},
	/* 4 to 1 */
	{
		{LN8411_VUSB_OVP, VUSB_OVP_DFT_4_1, 0xcf},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_4_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_4_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_4_1, 0x0},
		{LN8411_PMID2OUT_UVP, PMID2OUT_UVP_DFT_4_1, 0x9f},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
	},
	/* 1 to 2 */
	{
		{LN8411_VBAT_OVP, VBAT_OVP_DFT_1_2, 0x0},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_2_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_4_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_1_2, 0x0},
		{LN8411_PMID2OUT_UVP, PMID2OUT_UVP_DFT_1_2, 0x9f},
		{LN8411_IBUS_UCP, IBUS_UCP_DFT_1_2, 0x0},
		{LN8411_LION_COMP_CTRL_1, PMID_SWITCH_OK_DIS, 0x0},
		{LN8411_LION_COMP_CTRL_2, LN8411_VWPC_UVP_DIS, 0x0},
		{LN8411_LION_COMP_CTRL_4, LN8411_INFET_OFF_DET_DIS, 0x0},
	},
};

static u8 mode_settings_B0[3][9][3] = {
	/* 2 to 1 */
	{
		{LN8411_IBUS_UCP, IBUS_UCP_DFT_2_1, 0x3},
		{LN8411_VUSB_OVP, VUSB_OVP_DFT_2_1_B0, 0xcf},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_2_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_2_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_2_1, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
	},
	/* 4 to 1 */
	{
		{LN8411_IBUS_UCP, IBUS_UCP_DFT_4_1, 0x3},
		{LN8411_VUSB_OVP, VUSB_OVP_DFT_4_1_B0, 0xcf},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_4_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_4_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_4_1, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
		{0x0, 0x0, 0x0},
	},
	/* 1 to 2 */
	{
		{LN8411_VBAT_OVP, VBAT_OVP_DFT_1_2, 0x0},
		{LN8411_VWPC_OVP, VUSB_OVP_DFT_2_1, 0xcf},
		{LN8411_CFG_10, CFG_10_DFT_4_1, 0x1},
		{LN8411_IBUS_OCP, IBUS_OCP_DFT_1_2, 0x0},
		{LN8411_PMID2OUT_UVP, PMID2OUT_UVP_DFT_1_2, 0x9f},
		{LN8411_IBUS_UCP, IBUS_UCP_DFT_1_2_B0, 0x0},
		{LN8411_LION_COMP_CTRL_1, PMID_SWITCH_OK_DIS, 0x0},
		{LN8411_LION_COMP_CTRL_2, LN8411_VWPC_UVP_DIS, 0x0},
		{LN8411_LION_COMP_CTRL_4, LN8411_INFET_OFF_DET_DIS, 0x0},
	},
};

static int ln8411_hw_init(struct ln8411_charger *ln8411);
static int ln8411_irq_init(struct ln8411_charger *ln8411);
static int ln8411_start_1_2_mode(struct ln8411_charger *ln8411);
static int ln8411_stop_1_2_mode(struct ln8411_charger *ln8411);

static inline int conv_chg_mode(const struct ln8411_charger *ln8411, int val)
{
	return (ln8411->chg_mode == CHG_2TO1_DC_MODE) ? val * 2 : val * 4;
}

int get_chip_info(struct ln8411_charger *chg)
{
	unsigned int val;

	int err = regmap_read(chg->regmap, LN8411_DEVICE_ID, &val);
	if (err) {
		dev_err(chg->dev, "Error reading DEVICE_ID (%d)\n", err);
		return err;
	}

	chg->chip_info.device_id = val;

	err = regmap_read(chg->regmap, LN8411_BC_STS_C, &val);
	if (err) {
		dev_err(chg->dev, "Error reading CHIP_REV (%d)\n", err);
		return err;
	}

	chg->chip_info.chip_rev = (val & LN8411_CHIP_REV_MASK) >> LN8411_CHIP_REV_SHIFT;

	dev_info(chg->dev, "DeviceID: %02X, Chip Rev: %02X\n",	chg->chip_info.device_id,
		 chg->chip_info.chip_rev);

	return 0;
}

static ssize_t chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct ln8411_charger *chg = dev_get_drvdata(dev);

	ret = get_chip_info(chg);
	if (ret) {
		dev_err(dev, "Error while getting chip info\n");
		return ret;
	}

	ret = scnprintf(buf, PAGE_SIZE, "Chip Id : %#02X, Chip Rev: %#02X\n",
			chg->chip_info.device_id, chg->chip_info.chip_rev);
	return ret;
}

static DEVICE_ATTR_RO(chip_info);

static struct attribute *ln8411_attr_group[] = {
	&dev_attr_chip_info.attr,
	NULL
};

static int dump_all_regs(struct ln8411_charger *ln8411)
{
	u8 tmp[0x9c - LN8411_DEVICE_ID + 1];
	int i;
	int ret, bc_reg, bd_reg, be_reg;
	int adc_val;

	adc_val = ln8411_read_adc(ln8411, ADCCH_VBAT);
	dev_info(ln8411->dev, "VBAT ADC: %d\n", adc_val);

	ret = regmap_bulk_read(ln8411->regmap, LN8411_DEVICE_ID, &tmp, sizeof(tmp));
	for (i=0; i<sizeof(tmp); i++)
		dev_info(ln8411->dev, "Reg %#02x = %#02x\n", i, tmp[i]);

	ret = regmap_read(ln8411->regmap, 0xbc, &bc_reg);
	ret = regmap_read(ln8411->regmap, 0xbd, &bd_reg);
	ret = regmap_read(ln8411->regmap, 0xbe, &be_reg);
	dev_info(ln8411->dev, "Reg 0xbc = %#02x, 0xbd = %#02x, 0xbe = %#02x\n", bc_reg, bd_reg, be_reg);
	return ret;
}

static bool ln8411_is_reg(struct device *dev, unsigned int reg)
{
	switch(reg) {
	case 0x0 ... 0xbe:
		return true;
	default:
		return false;
	}
}

static struct regmap_config ln8411_regmap = {
	.name		= "ln8411",
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0xbe,
	.readable_reg = ln8411_is_reg,
	.volatile_reg = ln8411_is_reg,
};

static int read_reg(void *data, u64 *val)
{
	struct ln8411_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;

	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct ln8411_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;

	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		dev_err(chip->dev, "Couldn't write %#02x to %#02x rc = %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops_ln8411, read_reg, write_reg, "%#02llx\n");

static int ln8411_read_1_2_mode(void *data, u64 *val)
{
	struct ln8411_charger *ln8411 = data;

	*val = ln8411->chg_mode == CHG_1TO2_DC_MODE;
	return 0;
}

static int ln8411_write_1_2_mode(void *data, u64 val)
{
	struct ln8411_charger *ln8411 = data;
	int rc;

	if (val)
		rc = ln8411_start_1_2_mode(ln8411);
	else
		rc = ln8411_stop_1_2_mode(ln8411);
	if (rc) {
		dev_err(ln8411->dev, "Couldn't %s 1_2 mode\n", val ? "enable" :  "disable");
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ln8411_1_2_mode_ops, ln8411_read_1_2_mode, ln8411_write_1_2_mode, "%#02llx\n");
/* ------------------------------------------------------------------------ */

static int ln8411_set_lion_ctrl(const struct ln8411_charger *ln8411, enum ln8411_keys key)
{
	int ret;

	ret = regmap_write(ln8411->regmap, LN8411_LION_CTRL, (unsigned int)key);
	if (ret)
		dev_err(ln8411->dev, "Failed to set LION_CTRL: key: %d (%d)\n", key, ret);

	return ret;
}

static int __ln8411_get_adc__(struct ln8411_charger *ln8411,
			      const enum ln8411_adc_chan chan, u16 *val)
{
	unsigned int lsb, msb, reg;
	int ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error setting EN_SW_OVERRIDE (%d)\n", __func__, ret);
		goto error;
	}

	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CFG_2, LN8411_PAUSE_ADC_UPDATES);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error pausing ADC updates (%d)\n", __func__, ret);
		goto error;
	}

	reg = (2 * chan) + LN8411_IBUS_ADC1;

	ret = regmap_read(ln8411->regmap, reg, &msb);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error reading msb reg %#02x (%d)\n", __func__, reg, ret);
		goto resume_updates;
	}

	reg++;

	ret = regmap_read(ln8411->regmap, reg, &lsb);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error reading lsb reg %#02x (%d)\n", __func__, reg, ret);
		goto resume_updates;
	}

	*val = (msb << LN8411_REG_BITS) | lsb;

resume_updates:
	ret = regmap_clear_bits(ln8411->regmap, LN8411_ADC_CFG_2, LN8411_PAUSE_ADC_UPDATES);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error resuming ADC updates (%d)\n", __func__, ret);
		goto error;
	}

error:
	ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	return ret;
}

int ln8411_read_adc(struct ln8411_charger *ln8411,
			  const enum ln8411_adc_chan chan)
{
	int ret, intval;
	u16 val;

	ret = __ln8411_get_adc__(ln8411, chan, &val);
	if (ret)
		return ret;

	switch (chan) {
	case LN8411_ADC_CHAN_IBUS:
		intval = val * LN8411_IBUS_ADC_STEP_UA;
		break;
	case LN8411_ADC_CHAN_VBUS:
		intval = val * LN8411_VBUS_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VUSB:
		intval = val * LN8411_VUSB_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VWPC:
		intval = val * LN8411_VWPC_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VOUT:
		intval = val * LN8411_VOUT_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_VBAT:
		intval = val * LN8411_VBAT_ADC_STEP_UV;
		break;
	case LN8411_ADC_CHAN_IBAT:
		intval = val * LN8411_IBAT_ADC_STEP_UA;
		break;
	case LN8411_ADC_CHAN_TSBAT:
		intval = val;
		break;
	case LN8411_ADC_CHAN_TDIE:
		intval = val * LN8411_TDIE_STEP_DECIC;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(ln8411->dev, "%s: ADC Ch: %d = %d\n", __func__, chan, intval);

	return intval;
}

static int ln8411_read_sys_sts(struct ln8411_charger *ln8411, int *val)
{
	int ret, retries = 10;

	for (; retries; retries--) {
		ret = regmap_read(ln8411->regmap, LN8411_SYS_STS, val);
		if (!ret && *val)
			break;

		dev_dbg(ln8411->dev, "%s: retries: %d\n", __func__, retries);
		msleep(5);
	}

	return ret;
}

static int ln8411_set_vbat_ovp(struct ln8411_charger *ln8411, int val)
{
	unsigned int reg_code;
	int ret;

	val = clamp(val, LN8411_VBAT_OVP_MIN_UV, LN8411_VBAT_OVP_MAX_UV);

	reg_code = (val - LN8411_VBAT_OVP_OFFSET_UV) / LN8411_VBAT_OVP_STEP_UV;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_VBAT_OVP, LN8411_VBAT_OVP_MASK);
	if (ret)
		return ret;

	return regmap_update_bits(ln8411->regmap, LN8411_VBAT_OVP,
				 LN8411_VBAT_OVP_CFG_MASK, reg_code);
}

/* v float voltage (5 mV) resolution */
static int ln8411_set_vfloat(struct ln8411_charger *ln8411,
			      unsigned int v_float)
{
	int ret = 0;

	/* Temporary for A1 silicon: Use ADC */
	/* ret = ln8411_set_vbat_ovp(ln8411, v_float + VBAT_ALARM_CFG_DELTA); */
	ln8411->vfloat_reg = v_float;


	dev_info(ln8411->dev, "%s: v_float=%u\n", __func__, v_float);

	return ret;
}

static int ln8411_set_input_current(struct ln8411_charger *ln8411,
				     unsigned int iin)
{
	int ret = 0, val;

	/* round-up and increase one step */
	/* iin = iin + PD_MSG_TA_CUR_STEP + IBUS_ALARM_CFG_DELTA; */
	iin += PD_MSG_TA_CUR_STEP;
	val = LN8411_IIN_CFG(iin);
	/* Set IIN_CFG to one step higher */
	val = val + 1;

	/* Temporary for A1 silicon: Use ADC */
	/* ret = ln8411_set_ibus_ocp(ln8411, iin); */
	ln8411->iin_reg = val * LN8411_IIN_CFG_STEP;

	dev_info(ln8411->dev, "%s: iin=%d (%d)\n", __func__, iin, ret);

	return ret;
}

static inline bool ln8411_can_inc_ta_cur(struct ln8411_charger *ln8411)
{
	return ln8411->ta_cur + PD_MSG_TA_CUR_STEP < min(ln8411->ta_max_cur,
		ln8411->iin_cc + LN8411_TA_CUR_MAX_OFFSET);
}

/* Returns the enable or disable value. into 1 or 0. */
static int ln8411_get_charging_enabled(struct ln8411_charger *ln8411)
{
	int ret;
	unsigned int val;

	ret = regmap_read(ln8411->regmap, LN8411_CTRL1, &val);
	if (ret < 0)
		return ret;

	return (val & LN8411_CP_EN) != 0;
}


/* b/194346461 ramp down IIN */
static int ln8411_wlc_ramp_down_iin(struct ln8411_charger *ln8411,
				     struct power_supply *wlc_psy)
{
	const int ramp_down_step = LN8411_IIN_CFG_STEP;
	int ret = 0, iin;

	if (!ln8411->wlc_ramp_out_iin)
		return 0;

	iin = ln8411_input_current_limit(ln8411);
	for ( ; iin >= LN8411_IIN_CFG_MIN; iin -= ramp_down_step) {
		int iin_adc, wlc_iout = -1;

		iin_adc = ln8411_read_adc(ln8411, ADCCH_IIN);

		if (wlc_psy) {
			union power_supply_propval pro_val;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_ONLINE,
					&pro_val);
			if (ret < 0 || pro_val.intval != PPS_PSY_PROG_ONLINE)
				break;

			ret = power_supply_get_property(wlc_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&pro_val);
			if (ret == 0)
				wlc_iout = pro_val.intval;
		}

		ret = ln8411_set_input_current(ln8411, iin);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: ramp down iin=%d (%d)\n", __func__,
				iin, ret);
			break;
		}

		dev_dbg(ln8411->dev, "%s: iin_adc=%d, wlc_iout-%d ramp down iin=%d\n",
				__func__, iin_adc, wlc_iout, iin);
		msleep(ln8411->wlc_ramp_out_delay);
	}

	return ret;
}

/* b/194346461 ramp down VOUT */
#define WLC_VOUT_CFG_STEP	40000

/* the caller will set to vbatt * 4 */
static int ln8411_wlc_ramp_down_vout(struct ln8411_charger *ln8411,
				struct power_supply *wlc_psy)
{
	const int ramp_down_step = WLC_VOUT_CFG_STEP;
	union power_supply_propval pro_val;
	int vout = 0, vout_target = ln8411->wlc_ramp_out_vout_target;
	int ret, vbatt;

	while (true) {
		vbatt = ln8411_read_adc(ln8411, ADCCH_VBAT);
		if (vbatt <= 0) {
			dev_err(ln8411->dev, "%s: invalid vbatt %d\n", __func__, vbatt);
			break;
		}

		ret = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&pro_val);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: invalid vout %d\n", __func__, ret);
			break;
		}

		if (!ln8411->wlc_ramp_out_vout_target)
			vout_target = vbatt * 4;

		if (!vout)
			vout = pro_val.intval;
		if (vout < vout_target) {
			dev_dbg(ln8411->dev, "%s: underflow vout=%d, vbatt=%d (target=%d)\n",
				__func__, vout, vbatt, vout_target);
			return 0;
		}

		pro_val.intval = vout - ramp_down_step;

		dev_dbg(ln8411->dev, "%s: vbatt=%d, wlc_vout=%d->%d\n", __func__, vbatt,
			 vout, pro_val.intval);

		ret = power_supply_set_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&pro_val);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: cannot set vout %d\n", __func__, ret);
			break;
		}

		msleep(ln8411->wlc_ramp_out_delay);
		vout = pro_val.intval;
	}

	return -EIO;
}

static int ln8411_set_mode(struct ln8411_charger *ln8411, const int val)
{
	enum ln8411_modes mode;
	int ret;

	switch (val) {
	case CHG_2TO1_DC_MODE:
		mode = LN8411_FWD2TO1;
		break;
	case CHG_4TO1_DC_MODE:
		mode = LN8411_FWD4TO1;
		break;
	case CHG_1TO2_DC_MODE:
		mode = LN8411_REV1TO2;
		break;
	case CHG_NO_DC_MODE:
		mode = LN8411_FWD1TO1;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(ln8411->regmap, LN8411_CTRL4, LN8411_MODE_MASK, mode);
	if (ret)
		goto done;

	/* Workaround for A1 for 2:1 and 1:2 mode */
	if ((val == CHG_2TO1_DC_MODE || val == CHG_1TO2_DC_MODE)
	     && ln8411->chip_info.chip_rev == 1) {
		ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
		if (ret)
			goto done;

		ret = regmap_set_bits(ln8411->regmap, 0x93, 0x2);
		if (ret)
			goto done;

		ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
		if (ret)
			goto done;
	}

done:
	return ret;
}

static int ln8411_set_status_charging(struct ln8411_charger *ln8411)
{
	int ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_QB_EN);
	if (ret)
		return ret;

	msleep(30);

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_CP_EN);
	return ret;
}

static int ln8411_set_status_disable_charging(struct ln8411_charger *ln8411)
{
	int ret;
	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_CP_EN);
	if (ret)
		return ret;

	msleep(60);

	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_QB_EN);
	return ret;
}

static int ln8411_set_prot_by_chg_mode(const struct ln8411_charger *ln8411)
{
	int ret;
	int mode_idx, i;
	u8 (*mode_settings)[9][3];

	mode_idx = ln8411->chg_mode - 1;
	if (ln8411->chip_info.chip_rev == 1)
		mode_settings = mode_settings_A1;
	else
		mode_settings = mode_settings_B0;
	if (mode_idx < 0 || mode_idx >= CHG_1TO2_DC_MODE) {
		dev_info(ln8411->dev, "%s: Invalid mode: %d\n", __func__, mode_idx + 1);
		return -EINVAL;
	}

	/* unlock private register space */
	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error unlocking private reg (%d)\n", __func__, ret);
		goto error_done;
	}

	for (i = 0; i < sizeof(mode_settings[0]) / sizeof(mode_settings[0][0]); i++) {
		if (!mode_settings[mode_idx][i][0])
			continue;

		if (mode_settings[mode_idx][i][2])
			ret = regmap_update_bits(ln8411->regmap, mode_settings[mode_idx][i][0],
						 mode_settings[mode_idx][i][2],
						 mode_settings[mode_idx][i][1]);
		else
			ret = regmap_write(ln8411->regmap, mode_settings[mode_idx][i][0],
					    mode_settings[mode_idx][i][1]);
		if (ret) {
			dev_info(ln8411->dev, "Error setting reg mode: %d, reg: %#02x, val: %#02x (%d)\n",
				 ln8411->chg_mode, mode_settings[mode_idx][i][0],
				 mode_settings[mode_idx][i][1], ret);
			goto error_done;
		}
	}

error_done:
	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	if (ret)
		dev_info(ln8411->dev, "%s: Error locking private reg (%d)\n", __func__, ret);
	return ret;
}

/* call holding mutex_lock(&ln8411->lock); */
static int ln8411_set_charging(struct ln8411_charger *ln8411, bool enable)
{
	int ret;

	dev_dbg(ln8411->dev, "%s: enable=%d ta_type=%d\n", __func__,  enable, ln8411->ta_type);

	if (enable && ln8411_get_charging_enabled(ln8411) == enable) {
		dev_dbg(ln8411->dev, "%s: no op, already enabled\n", __func__);
		return 0;
	}

	if (enable) {
		int val;

		/* Integration guide V1.0 Section 5.3 */
		/* check power source present */
		ret = regmap_read(ln8411->regmap, LN8411_INT_STAT, &val);
		if (ret)
			goto error;
		val &= (LN8411_VWPC_INSERT_STAT | LN8411_VUSB_INSERT_STAT);
		if (!val) {
			dev_info(ln8411->dev, "No power source. Not enabling charging\n");
			goto error;
		}

		/* Ensure we are in standby to start charging */
		ret = ln8411_read_sys_sts(ln8411, &val);
		if (ret || !(val & LN8411_STANDBY_STS)) {
			dev_info(ln8411->dev, "%s: ret=%d Not in standby SYS_STS: %#02x\n",
				  __func__, ret, val);
			goto error;
		}

		/* Start charging */
		ret = ln8411_set_status_charging(ln8411);
		if (ret < 0)
			goto error;
	} else {
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			struct power_supply *wlc_psy;

			wlc_psy = ln8411_get_rx_psy(ln8411);
			if (wlc_psy) {
				int ret;

				ret = ln8411_wlc_ramp_down_iin(ln8411, wlc_psy);
				if (ret < 0)
					dev_err(ln8411->dev, "cannot ramp out iin (%d)\n", ret);

				ret = ln8411_wlc_ramp_down_vout(ln8411, wlc_psy);
				if (ret < 0)
					dev_err(ln8411->dev, "cannot ramp out vout (%d)\n", ret);
			}
		}

		/* Integration guide V1.0 Section 5.4 */
		/* turn off charging */
		ret = ln8411_set_status_disable_charging(ln8411);
		if (ret < 0)
			goto error;
	}

error:
	if (ret)
		dev_info(ln8411->dev, "%s: Error: ret:%d\n", __func__, ret);
	else
		dev_dbg(ln8411->dev, "%s: End\n", __func__);
	return ret;
}

static int ln8411_check_state(struct ln8411_charger *ln8411, int loglevel)
{
	int ret;
	unsigned int int_flag = 0, int_stat = 0, comp_flag0 = 0, comp_flag1 = 0, ctrl5 = 0;

	ret = regmap_read(ln8411->regmap, LN8411_INT_FLAG, &int_flag);
	ret |= regmap_read(ln8411->regmap, LN8411_INT_STAT, &int_stat);
	ret |= regmap_read(ln8411->regmap, LN8411_COMP_FLAG0, &comp_flag0);
	ret |= regmap_read(ln8411->regmap, LN8411_COMP_FLAG1, &comp_flag1);
	ret |= regmap_read(ln8411->regmap, LN8411_CTRL5, &ctrl5);

	logbuffer_prlog(ln8411, loglevel,
			"%s: ret: %d, INT_FLAG: %#02x, STAT: %#02x, COMP_FLAG0: %#02x, COMP_FLAG1: %#02x\n",
			__func__, ret, int_flag, int_stat, comp_flag0, comp_flag1);
	logbuffer_prlog(ln8411, loglevel, "%s: CTRL5: %#02x\n", __func__, ctrl5);

	return ret;
}

static bool ln8411_err_is_ucp(struct ln8411_charger *ln8411, bool *retries)
{
	*retries = !!ln8411->ibus_ucp_retry_cnt;
	return ln8411->error == LN8411_ERROR_UCP;
}

static bool ln8411_err_is_low_batt(struct ln8411_charger *ln8411, bool *retries)
{
	*retries = !!ln8411->low_batt_retry_cnt;
	return ln8411->error == LN8411_ERROR_LOW_VBATT;
}

static int ln8411_check_not_active(struct ln8411_charger *ln8411, int loglevel)
{
	int ret, rc = -EINVAL;
	unsigned int reg;
	u8 safety_sts[4];

	ret = regmap_bulk_read(ln8411->regmap, LN8411_SAFETY_STS, safety_sts, 4);
	if (ret < 0)
		goto done;

	ret = ln8411_read_sys_sts(ln8411, &reg);
	if (ret < 0)
		goto done;

	if (reg & LN8411_STANDBY_STS) {
		if (safety_sts[0] & LN8411_REV_IBUS_LATCHED) {
			rc = -EAGAIN;
			ln8411->error = LN8411_ERROR_UCP;
		} else {
			rc = -EINVAL;
			ln8411->error = LN8411_ERROR_NOT_ACTIVE;
		}
		logbuffer_prlog(ln8411, loglevel, "%s: in standby (%d)\n", __func__, rc);
	} else if (reg & LN8411_SHUTDOWN_STS) {
		rc = -EINVAL;
		logbuffer_prlog(ln8411, loglevel, "%s: in shutdown\n", __func__);
	} else {
		rc = 0;
	}

done:
	logbuffer_prlog(ln8411, loglevel,
			"%s: ret: %d, LN8411_SAFETY_STS 0x99:%#02x, 0x9a:%#02x, 0x9b:%#02x, 0x9c:%#02x\n",
			__func__, ret, safety_sts[0], safety_sts[1], safety_sts[2], safety_sts[3]);

	return rc;
}

int ln8411_check_active(struct ln8411_charger *ln8411)
{
	int ret;
	unsigned int reg, val;

	/* Integration guide V1.0 Section 5.3.4 */
	ret = ln8411_read_sys_sts(ln8411, &reg);
	if (ret < 0) {
		dev_err(ln8411->dev, "Error reading LN8411_SYS_STS err: %d\n", ret);
		return ret;
	}

	if (ln8411->chg_mode == CHG_4TO1_DC_MODE)
		val = (reg == (LN8411_PMID_SWITCH_OK_STS | LN8411_INFET_OK_STS
		       | LN8411_SWITCHING41_ACTIVE_STS));
	else if (ln8411->chg_mode == CHG_2TO1_DC_MODE || ln8411->chg_mode == CHG_1TO2_DC_MODE)
		val = (reg == (LN8411_PMID_SWITCH_OK_STS | LN8411_INFET_OK_STS
		       | LN8411_SWITCHING21_ACTIVE_STS));
	else
		val = 0;

	if (!val) {
		if (ln8411->charging_state != DC_STATE_NO_CHARGING)
			dev_err(ln8411->dev, "%s: CP Not switching LN8411_SYS_STS: %#02X\n",
				__func__, reg);
		return -EINVAL;
	}

	return 1;
}

/*
 * Check Active status, 0 is active (or in RCP), <0 indicates a problem.
 * The function is called from different contexts/functions, errors are fatal
 * (i.e. stop charging) from all contexts except when this is called from
 * ln8411_check_active_state().
 *
 * Other contexts:
 * . ln8411_charge_adjust_ccmode
 * . ln8411_charge_ccmode
 * . ln8411_charge_start_cvmode
 * . ln8411_charge_cvmode
 * . ln8411_adjust_ta_voltage
 * . ln8411_adjust_rx_voltage
 * . ln8411_adjust_ta_current
 * call holding mutex_lock(&ln8411->lock)
 */
static int ln8411_check_error(struct ln8411_charger *ln8411)
{
	int ret = -EINVAL, vbatt;
	const int debounce_cnt = ln8411->ibus_ucp_debounce_cnt;

	/* LN8411 is active state */
	if (ln8411_check_active(ln8411) == 1) {
		if (ln8411->ibus_ucp_debounce_cnt &&
		    (ln8411->ibus_ucp_retry_cnt != LN8411_MAX_IBUS_UCP_RETRY_CNT)) {
			ln8411->ibus_ucp_debounce_cnt--;
		} else if (!ln8411->ibus_ucp_debounce_cnt) {
			ln8411->ibus_ucp_retry_cnt = LN8411_MAX_IBUS_UCP_RETRY_CNT;
			ln8411->ibus_ucp_debounce_cnt = LN8411_MAX_IBUS_UCP_DEBOUNCE_COUNT;
		}
		ln8411->error = LN8411_ERROR_NONE;
		ln8411->low_batt_retry_cnt = LN8411_MAX_LOW_BATT_RETRY_CNT;
		dev_dbg(ln8411->dev, "%s: Active Status ok. debounce_cnt:%d->%d\n", __func__,
			debounce_cnt, ln8411->ibus_ucp_debounce_cnt);

		return 0;
	}

	/* LN8411 is charging */
	/* Check whether the battery voltage is over the minimum */
	vbatt = ln8411_read_adc(ln8411, ADCCH_VBAT);
	if (vbatt <= LN8411_DC_VBAT_MIN)
		/* Abnormal battery level */
		dev_err(ln8411->dev, "%s: Error abnormal battery voltage=%d\n",	__func__, vbatt);

	ln8411_check_state(ln8411, LOGLEVEL_ERR);
	ret = ln8411_check_not_active(ln8411, LOGLEVEL_ERR);

	/*
	 * Sometimes battery driver might call set_property function
	 * to stop charging during msleep. At this case, charging
	 * state would change DC_STATE_NO_CHARGING. LN8411 should
	 * stop checking RCP condition and exit timer_work
	 */
	if (ln8411->charging_state == DC_STATE_NO_CHARGING)
		dev_err(ln8411->dev, "%s: other driver forced stop\n", __func__);

	dev_dbg(ln8411->dev, "%s: Not Active Status=%d\n", __func__, ret);
	return ret;
}

static int ln8411_get_iin(struct ln8411_charger *ln8411, int *iin)
{
	const int offset = 0;
	int temp;

	temp = ln8411_read_adc(ln8411, ADCCH_IIN);
	if (temp < 0)
		return temp;

	if (temp < offset)
		temp = offset;

	*iin = conv_chg_mode(ln8411, temp - offset);
	return 0;
}

/* only needed for logging */
static int ln8411_get_batt_info(struct ln8411_charger *ln8411, int info_type, int *info)
{
	union power_supply_propval val;
	enum power_supply_property psp;
	int ret;

	if (!ln8411->batt_psy)
		ln8411->batt_psy = power_supply_get_by_name("battery");
	if (!ln8411->batt_psy)
		return -EINVAL;

	if (info_type == BATT_CURRENT)
		psp = POWER_SUPPLY_PROP_CURRENT_NOW;
	else
		psp = POWER_SUPPLY_PROP_VOLTAGE_NOW;

	ret = power_supply_get_property(ln8411->batt_psy, psp, &val);
	if (ret == 0)
		*info = val.intval;

	return ret;
}

/* only needed for logging */
static int ln8411_get_ibatt(struct ln8411_charger *ln8411, int *info)
{
	return ln8411_get_batt_info(ln8411, BATT_CURRENT, info);
}

static int ln8411_get_current_adcs(struct ln8411_charger *ln8411, int *pibat, int *picn, int *piin)
{
	int rc = ln8411_get_ibatt(ln8411, pibat);
	if (rc)
		goto error;

	rc = ln8411_get_iin(ln8411, picn);
	if (rc)
		goto error;

	*piin = ln8411_read_adc(ln8411, ADCCH_IIN);
	return 0;

error:
	logbuffer_prlog(ln8411, LOGLEVEL_ERR, "%s: Error: rc=%d", __func__, rc);
	return rc;
}

static void ln8411_prlog_state(struct ln8411_charger *ln8411, const char *fn)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int vbat = ln8411_read_adc(ln8411, ADCCH_VBAT);

	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		goto error;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: vbat=%d, iin=%d, iin_cc=%d, icn=%d ibat=%d, cc_max=%d rc=%d",
			fn, vbat, iin, ln8411->iin_cc, icn, ibat, ln8411->cc_max, rc);
	return;

error:
	dev_info(ln8411->dev, "Error reading ibatt or icn: rc: %d, ibatt: %d, icn: %d\n",
		 rc, ibat, icn);
}

static int ln8411_read_status(struct ln8411_charger *ln8411)
{
	int ret = 0;
	unsigned int reg_val;

	ret = regmap_read(ln8411->regmap, LN8411_FAULT3_STS, &reg_val);
	if (ret < 0) {
		dev_err(ln8411->dev, "Error reading LN8411_FAULT3_STS: %d\n", ret);
		return ret;
	}

	/* Temporary for A1 silicon: Use ADC
	if (reg_val & LN8411_IBUS_ALARM_STS) {
		ret = STS_MODE_IIN_LOOP;
	} else if (reg_val & LN8411_VBAT_ALARM_STS) {
		ret = STS_MODE_VFLT_LOOP;
	} else {
		ret = STS_MODE_LOOP_INACTIVE;
	}
	*/

	if (ln8411_read_adc(ln8411, ADCCH_IIN) >= ln8411->iin_reg)
		ret = STS_MODE_IIN_LOOP;
	else if (ln8411_read_adc(ln8411, ADCCH_VBAT) >= ln8411->vfloat_reg)
		ret = STS_MODE_VFLT_LOOP;
	else
		ret = STS_MODE_LOOP_INACTIVE;;

	return ret;
}

static int ln8411_const_charge_voltage(struct ln8411_charger *ln8411);

static int ln8411_check_status(struct ln8411_charger *ln8411)
{
	int icn = -EINVAL, ibat = -EINVAL, vbat = -EINVAL;
	int rc, status;

	status = ln8411_read_status(ln8411);
	if (status < 0)
		goto error;

	rc = ln8411_get_iin(ln8411, &icn);
	if (rc)
		goto error;

	rc = ln8411_get_batt_info(ln8411, BATT_CURRENT, &ibat);
	if (rc)
		goto error;

	rc = ln8411_get_batt_info(ln8411, BATT_VOLTAGE, &vbat);

error:
	dev_dbg(ln8411->dev, "%s: status=%d rc=%d icn:%d ibat:%d delta_c=%d, vbat:%d, fv:%d, cc_max:%d\n",
		 __func__, status, rc, icn, ibat, icn - ibat, vbat,
		 ln8411->fv_uv, ln8411->cc_max);

	return status;
}

/* hold mutex_lock(&ln8411->lock); */
static int ln8411_recover_ta(struct ln8411_charger *ln8411)
{
	int ret;

	if (ln8411->ftm_mode)
		return 0;

	if (ln8411->ta_type == TA_TYPE_WIRELESS) {
		ln8411->ta_vol = 0; /* set to a value to change rx vol */
		ret = ln8411_send_rx_voltage(ln8411, MSG_REQUEST_FIXED_PDO);
	} else {
		/* TODO: recover TA to value before handoff, or use DT */
		ln8411->ta_vol = 9000000;
		ln8411->ta_cur = 2200000;
		ln8411->ta_objpos = 1; /* PDO1 - fixed 5V */
		ret = ln8411_send_pd_message(ln8411, MSG_REQUEST_FIXED_PDO);
	}

	/* will not be able to recover if TA is offline */
	if (ret < 0)
		dev_dbg(ln8411->dev, "%s: cannot recover TA (%d)\n", __func__, ret);

	return 0;
}

/* Stop Charging */
static int ln8411_stop_charging(struct ln8411_charger *ln8411)
{
	int ret = 0;

	/* mark the end with \n in logbuffer */
	logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
			"%s: ln8411->charging_state=%d ret=%d\n",
			__func__, ln8411->charging_state, ret);

	mutex_lock(&ln8411->lock);

	/* Check the current state */
	if (ln8411->charging_state == DC_STATE_NO_CHARGING)
		goto done;

	/* Stop Direct charging  */
	cancel_delayed_work(&ln8411->timer_work);
	cancel_delayed_work(&ln8411->pps_work);
	ln8411->timer_id = TIMER_ID_NONE;
	ln8411->timer_period = 0;

	/* Clear parameter */
	if (ln8411->charging_state != DC_STATE_ERROR)
		ln8411->charging_state = DC_STATE_NO_CHARGING;
	ln8411->ret_state = DC_STATE_NO_CHARGING;
	ln8411->prev_iin = 0;
	ln8411->prev_inc = INC_NONE;
	ln8411->chg_mode = CHG_NO_DC_MODE;

	/* restore to config */
	ln8411->pdata->iin_cfg = ln8411->pdata->iin_cfg_max;

	/*
	 * Clear charging configuration
	 * TODO: use defaults when these are negative or zero at startup
	 * NOTE: cc_max is twice of IIN + headroom
	 */
	ln8411->cc_max = -1;
	ln8411->fv_uv = -1;

	/* Clear requests for new Vfloat and new IIN */
	ln8411->new_vfloat = 0;
	ln8411->new_iin = 0;

	/* used to start DC and during errors */
	ln8411->retry_cnt = 0;

	ln8411->prev_ta_cur = 0;
	ln8411->prev_ta_vol = 0;

	/* close stats */
	ln8411_chg_stats_done(&ln8411->chg_data, ln8411);
	ln8411_chg_stats_dump(ln8411);

	/* TODO: something here to prep TA for the switch */

	ret = ln8411_set_charging(ln8411, false);
	if (ret < 0)
		dev_err(ln8411->dev, "%s: Error-set_charging(main)\n", __func__);

	/* Integration guide V1.0 Section 4 - reinitialize on stop charging */
	ln8411->hw_init_done = false;
	if (!ln8411_hw_init(ln8411))
		ln8411->hw_init_done = true;

	/* stop charging and recover TA voltage */
	if (ln8411->mains_online == true)
		ln8411_recover_ta(ln8411);

	power_supply_changed(ln8411->mains);

done:
	mutex_unlock(&ln8411->lock);
	__pm_relax(ln8411->monitor_wake_lock);
	dev_dbg(ln8411->dev, "%s: END, ret=%d\n", __func__, ret);
	return ret;
}

#define FCC_TOLERANCE_RATIO		99
#define FCC_POWER_INCREASE_THRESHOLD	99

/*
 * Compensate TA current for the target input current called from
 * ln8411_charge_ccmode() when loop becomes not active.
 *
 * ln8411_charge_ccmode() ->
 * 	-> ln8411_set_rx_voltage_comp()
 * 	-> ln8411_set_ta_voltage_comp()
 * 	-> ln8411_set_ta_current_comp2()
 *
 * NOTE: call holding mutex_lock(&ln8411->lock);
 */
static int ln8411_set_ta_current_comp(struct ln8411_charger *ln8411)
{
	const int iin_high = ln8411->iin_cc + ln8411->pdata->iin_cc_comp_offset;
	const int iin_low = ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d prev_iin=%d",
			__func__, iin, iin_low, ln8411->iin_cc, iin_high,
			icn, ibat, ln8411->cc_max, rc,
			ln8411->prev_iin);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {

		/* TA current is higher than the target input current */
		if (ln8411->ta_cur > ln8411->iin_cc) {
			/* TA current is over than IIN_CC */
			/* Decrease TA current (50mA) */
			ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1: ta_cur=%u",
					ln8411->ta_cur);

		/* TA current is already less than IIN_CC */
		/* Compara IIN_ADC with the previous IIN_ADC */
		} else if (iin < (ln8411->prev_iin - LN8411_IIN_ADC_OFFSET)) {
			/* Assume that TA operation mode is CV mode */
			/* Decrease TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2-1: ta_vol=%u",
					ln8411->ta_vol);
		} else {
			/* Assume TA operation mode is CL mode */
			/* Decrease TA current (50mA) */
			ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2-2: ta_cur=%u",
					ln8411->ta_cur);
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

	} else if (iin < iin_low) {

		/* compare IIN ADC with previous IIN ADC + 20mA */
		if (iin > (ln8411->prev_iin + LN8411_IIN_ADC_OFFSET)) {
			/*
			 * TA voltage is not enough to supply the operating
			 * current of RDO: increase TA voltage
			 */

			/* Compare TA max voltage */
			if (ln8411->ta_vol == ln8411->ta_max_vol) {
				/* TA voltage is already the maximum voltage */
				/* Compare TA max current */
				if (!ln8411_can_inc_ta_cur(ln8411)) {
					/* TA voltage and current are at max */
					logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
							"End1: ta_vol=%u, ta_cur=%u",
							ln8411->ta_vol, ln8411->ta_cur);

					/* Set timer */
					ln8411->timer_id = TIMER_CHECK_CCMODE;
					ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
				} else {
					/* Increase TA current (50mA) */
					ln8411->ta_cur = ln8411->ta_cur + PD_MSG_TA_CUR_STEP;

					logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
							"Cont3: ta_cur=%u",
							ln8411->ta_cur);

					/* Send PD Message */
					ln8411->timer_id = TIMER_PDMSG_SEND;
					ln8411->timer_period = 0;

					/* Set TA increment flag */
					ln8411->prev_inc = INC_TA_CUR;
				}
			} else {
				/* Increase TA voltage (20mV) */
				ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"Cont4: ta_vol=%u", ln8411->ta_vol);

				/* Send PD Message */
				ln8411->timer_id = TIMER_PDMSG_SEND;
				ln8411->timer_period = 0;

				/* Set TA increment flag */
				ln8411->prev_inc = INC_TA_VOL;
			}

		/* TA current is lower than the target input current */
		/* Check the previous TA increment */
		} else if (ln8411->prev_inc == INC_TA_VOL) {
			/*
			 * The previous increment is TA voltage, but
			 * input current does not increase.
			 */

			/* Try to increase TA current */
			/* Compare TA max current */
			if (!ln8411_can_inc_ta_cur(ln8411)) {

				/* TA current is already the maximum current */
				/* Compare TA max voltage */
				if (ln8411->ta_vol == ln8411->ta_max_vol) {
					/*
					 * TA voltage and current are already
					 * the maximum values
					 */
					logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
							"End2: ta_vol=%u, ta_cur=%u",
							ln8411->ta_vol, ln8411->ta_cur);

					ln8411->timer_id = TIMER_CHECK_CCMODE;
					ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
				} else {
					/* Increase TA voltage (20mV) */
					ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP;
					logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
							"Cont5: ta_vol=%u",
							ln8411->ta_vol);

					/* Send PD Message */
					ln8411->timer_id = TIMER_PDMSG_SEND;
					ln8411->timer_period = 0;

					/* Set TA increment flag */
					ln8411->prev_inc = INC_TA_VOL;
				}
			} else {
				const unsigned int ta_cur = ln8411->ta_cur +
							    PD_MSG_TA_CUR_STEP;

				/* Increase TA current (50mA) */
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"Cont6: ta_cur=%u->%u",
						ln8411->ta_cur, ta_cur);

				ln8411->ta_cur = ln8411->ta_cur + PD_MSG_TA_CUR_STEP;
				ln8411->timer_id = TIMER_PDMSG_SEND;
				ln8411->timer_period = 0;

				ln8411->prev_inc = INC_TA_CUR;
			}

		/*
		 * The previous increment was TA current, but input current
		 * did not increase. Try to increase TA voltage.
		 */
		} else if (ln8411->ta_vol == ln8411->ta_max_vol) {
			/* TA voltage is already the maximum voltage */

			/* Compare TA maximum current */
			if (!ln8411_can_inc_ta_cur(ln8411)) {
				/*
				* TA voltage and current are already at the
				 * maximum values
				 */
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"End3: ta_vol=%u, ta_cur=%u",
						 ln8411->ta_vol, ln8411->ta_cur);

				ln8411->timer_id = TIMER_CHECK_CCMODE;
				ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
			} else {
				/* Increase TA current (50mA) */
				ln8411->ta_cur = ln8411->ta_cur + PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"Cont7: ta_cur=%u", ln8411->ta_cur);

				/* Send PD Message */
				ln8411->timer_id = TIMER_PDMSG_SEND;
				ln8411->timer_period = 0;

				/* Set TA increment flag */
				ln8411->prev_inc = INC_TA_CUR;
			}
		} else {
			/* Increase TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"Comp. Cont8: ta_vol=%u->%u",
					ln8411->ta_vol, ln8411->ta_vol);

			/* Send PD Message */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;

			/* Set TA increment flag */
			ln8411->prev_inc = INC_TA_VOL;
		}

	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"Comp. End4(valid): ta_vol=%u, ta_cur=%u",
				ln8411->ta_vol, ln8411->ta_cur);
		/* Set timer */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = LN8411_CCMODE_CHECK1_T;

		/* b/186969924: reset increment state on valid */
		ln8411->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	ln8411->prev_iin = iin;
	return 0;
}

/*
 * max iin given cc_max and iin_cfg.
 * TODO: maybe use pdata->iin_cfg if cc_max is zero or negative.
 */
static int ln8411_get_iin_max(const struct ln8411_charger *ln8411, int cc_max)
{
	const int cc_limit = ln8411->pdata->iin_max_offset +
			     cc_max / conv_chg_mode(ln8411, 1);
	int iin_max;

	iin_max = min(ln8411->pdata->iin_cfg_max, (unsigned int)cc_limit);

	dev_dbg(ln8411->dev, "%s: iin_max=%d iin_cfg=%u iin_cfg_max=%d cc_max=%d cc_limit=%d\n",
		 __func__, iin_max, ln8411->pdata->iin_cfg,
		 ln8411->pdata->iin_cfg_max, cc_max, cc_limit);

	return iin_max;
}

/* Compensate TA current for constant power mode */
/* hold mutex_lock(&ln8411->lock), schedule on return 0 */
static int ln8411_set_ta_current_comp2(struct ln8411_charger *ln8411)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	/* IIN = IBAT+SYSLOAD */
	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			ln8411->iin_cc - LN8411_IIN_CC_COMP_OFFSET_CP,
			ln8411->iin_cc,
			ln8411->iin_cc + LN8411_IIN_CC_COMP_OFFSET_CP,
			ln8411->pdata->iin_cfg,
			icn, ibat, ln8411->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (ln8411->pdata->iin_cfg + ln8411->pdata->iin_cc_comp_offset)) {
		/* TA current is higher than the target input current limit */
		ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;

		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	} else if (iin < (ln8411->iin_cc - LN8411_IIN_CC_COMP_OFFSET_CP)) {

		/* TA current is lower than the target input current */
		/* IIN_ADC < IIN_CC -20mA */
		if (ln8411->ta_vol == ln8411->ta_max_vol) {
			const int iin_cc_lb = ln8411->iin_cc -
				      ln8411->pdata->iin_cc_comp_offset;

			/* Check IIN_ADC < IIN_CC - 50mA */
			if (iin < iin_cc_lb) {
				unsigned int ta_max_vol = 0;
				unsigned int iin_apdo;
				unsigned int val;

				if (ln8411->chg_mode == CHG_4TO1_DC_MODE)
					ta_max_vol =
						   ln8411->pdata->ta_max_vol_4_1 * CHG_4TO1_DC_MODE;
				else if (ln8411->chg_mode == CHG_2TO1_DC_MODE)
					ta_max_vol =
						   ln8411->pdata->ta_max_vol_2_1 * CHG_2TO1_DC_MODE;

				/* Set new IIN_CC to IIN_CC - 50mA */
				ln8411->iin_cc = ln8411->iin_cc -
					  ln8411->pdata->iin_cc_comp_offset;

				/* Set new TA_MAX_VOL to TA_MAX_PWR/IIN_CC */
				/* Adjust new IIN_CC with APDO resolution */
				iin_apdo = ln8411->iin_cc / PD_MSG_TA_CUR_STEP;
				iin_apdo = iin_apdo * PD_MSG_TA_CUR_STEP;
				/* in mV */
				val = ln8411->ta_max_pwr / (iin_apdo / ln8411->chg_mode / 1000);
				/* Adjust values with APDO resolution(20mV) */
				val = val * 1000 / PD_MSG_TA_VOL_STEP;
				val = val * PD_MSG_TA_VOL_STEP; /* uV */

				/* Set new TA_MAX_VOL */
				ln8411->ta_max_vol = min(val, ta_max_vol);

				/* Increase TA voltage(40mV) */
				ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP * 2;

				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"Cont1: ta_vol=%u",
						ln8411->ta_vol);

				/* Send PD Message */
				ln8411->timer_id = TIMER_PDMSG_SEND;
				ln8411->timer_period = 0;
			} else {
				/* Wait for next current step compensation */
				/* IIN_CC - 50mA < IIN ADC < IIN_CC - 20mA */
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"Comp.(wait): ta_vol=%u",
						ln8411->ta_vol);

				/* Set timer */
				ln8411->timer_id = TIMER_CHECK_CCMODE;
				ln8411->timer_period = LN8411_CCMODE_CHECK2_T;
			}
		} else {
			/* Increase TA voltage(40mV) */
			ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP * 2;
			if (ln8411->ta_vol > ln8411->ta_max_vol)
				ln8411->ta_vol = ln8411->ta_max_vol;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
					ln8411->ta_vol);

			/* Send PD Message */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CFG + 50mA */
		dev_dbg(ln8411->dev, "End(valid): ta_vol=%u\n", ln8411->ta_vol);

		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = LN8411_CCMODE_CHECK2_T;

		/* b/186969924: reset increment state on valid */
		ln8411->prev_inc = INC_NONE;
	}

	/* Save previous iin adc */
	ln8411->prev_iin = iin;
	return 0;
}

/* Compensate TA voltage for the target input current */
/* hold mutex_lock(&ln8411->lock), schedule on return 0 */
static int ln8411_set_ta_voltage_comp(struct ln8411_charger *ln8411)
{
	const int iin_high = ln8411->iin_cc + ln8411->pdata->iin_cc_comp_offset;
	const int iin_low = ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset;
	const int ibat_limit = (ln8411->cc_max * FCC_POWER_INCREASE_THRESHOLD) / 100;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);
	dev_dbg(ln8411->dev, "%s: = charging_state=%u == \n", __func__,
		 ln8411->charging_state);

	/* IIN = IBAT+SYSLOAD */
	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, ln8411->iin_cc, iin_high,
			icn, ibat, ln8411->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > iin_high) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				ln8411->ta_vol);

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

	} else if (iin < ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset) {

		/* TA current is lower than the target input current */
		/* Compare TA max voltage */
		if (ln8411->ta_vol == ln8411->ta_max_vol) {
			/* TA is already at maximum voltage */
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,"End1(max TA vol): ta_vol=%u",
					ln8411->ta_vol);

			/* Set timer */
			/* Check the current charging state */
			if (ln8411->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				ln8411->timer_id = TIMER_CHECK_CCMODE;
				ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				ln8411->timer_id = TIMER_CHECK_CVMODE;
				ln8411->timer_period = LN8411_CVMODE_CHECK_T;
			}
		} else {
			const unsigned int ta_vol = ln8411->ta_vol;

			/* Increase TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2: ta_vol:%u->%u",
					ta_vol, ln8411->ta_vol);

			/* Send PD Message */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End(valid): ta_vol=%u low_ibat=%d\n",
				ln8411->ta_vol, ibat < ibat_limit);

		/* Check the current charging state */
		if (ln8411->charging_state == DC_STATE_CC_MODE) {
			ln8411->timer_id = TIMER_CHECK_CCMODE;
			ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
		} else {
			ln8411->timer_id = TIMER_CHECK_CVMODE;
			ln8411->timer_period = LN8411_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/* hold mutex_lock(&ln8411->lock), schedule on return 0 */
static int ln8411_set_rx_voltage_comp(struct ln8411_charger *ln8411)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);

	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin,
			ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset,
			ln8411->iin_cc,
			ln8411->iin_cc + ln8411->pdata->iin_cc_comp_offset,
			icn, ibat, ln8411->cc_max, rc);
	if (iin < 0)
		return iin;

	/* Compare IIN ADC with target input current */
	if (iin > (ln8411->iin_cc + ln8411->pdata->iin_cc_comp_offset)) {

		/* RX current is higher than the target input current */
		ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1: rx_vol=%u",
				ln8411->ta_vol);

		/* Set RX Voltage */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

	} else if (iin < (ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset)) {

		/* RX current is lower than the target input current */
		/* Compare RX max voltage */
		if (ln8411->ta_vol == ln8411->ta_max_vol) {

			/* TA current is already the maximum voltage */
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"End1(max RX vol): rx_vol=%u",
					ln8411->ta_vol);

			/* Check the current charging state */
			if (ln8411->charging_state == DC_STATE_CC_MODE) {
				/* CC mode */
				ln8411->timer_id = TIMER_CHECK_CCMODE;
				ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
			} else {
				/* CV mode */
				ln8411->timer_id = TIMER_CHECK_CVMODE;
				ln8411->timer_period = LN8411_CVMODE_CHECK_T;
			}
		} else {
			/* Increase RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol + WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2: rx_vol=%u",
					ln8411->ta_vol);

			/* Set RX Voltage */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End(valid): rx_vol=%u",
				ln8411->ta_vol);

		if (ln8411->charging_state == DC_STATE_CC_MODE) {
			ln8411->timer_id = TIMER_CHECK_CCMODE;
			ln8411->timer_period = LN8411_CCMODE_CHECK1_T;
		} else {
			ln8411->timer_id = TIMER_CHECK_CVMODE;
			ln8411->timer_period = LN8411_CVMODE_CHECK_T;
		}
	}

	return 0;
}

/*
 * iin limit for the adapter for the chg_mode
 * Minimum between the configuration, cc_max (scaled with offset) and the
 * adapter capabilities.
 */
static int ln8411_get_iin_limit(const struct ln8411_charger *ln8411)
{
	int iin_cc;

	iin_cc = ln8411_get_iin_max(ln8411, ln8411->cc_max);
	if (ln8411->ta_max_cur < iin_cc)
		iin_cc = ln8411->ta_max_cur;

	dev_dbg(ln8411->dev, "%s: iin_cc=%d ta_max_cur=%u, chg_mode=%d\n", __func__,
		 iin_cc, ln8411->ta_max_cur, ln8411->chg_mode);

	return iin_cc;
}

/* recalculate ->ta_vol looking at demand (cc_max) */
static int ln8411_set_wireless_dc(struct ln8411_charger *ln8411, int vbat)
{
	unsigned long val;

	ln8411->iin_cc = ln8411_get_iin_limit(ln8411);

	ln8411->ta_vol = 2 * vbat * ln8411->chg_mode +	LN8411_WLC_VOL_PRE_OFFSET;

	/* RX voltage resolution is 100mV */
	val = ln8411->ta_vol / WCRX_VOL_STEP;
	ln8411->ta_vol = val * WCRX_VOL_STEP;
	/* Set RX voltage to MIN[RX voltage, RX_MAX_VOL*chg_mode] */
	ln8411->ta_vol = min(ln8411->ta_vol, ln8411->ta_max_vol);

	/* ta_cur is ignored */
	logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_max_vol=%d", __func__,
			ln8411->iin_cc, ln8411->ta_vol, ln8411->ta_max_vol);

	return 0;
}

/* recalculate ->ta_vol and ->ta_cur looking at demand (cc_max) */
static int ln8411_set_wired_dc(struct ln8411_charger *ln8411, int vbat)
{
	const unsigned long ta_max_vol = ln8411->ta_max_vol;
	unsigned long val;
	int iin_cc;

	ln8411->iin_cc = ln8411_get_iin_limit(ln8411);
	/* Update OCP_WARN_THRES as chg_mode might have changed to 2:1 */
	ln8411_set_input_current(ln8411, ln8411->iin_cc);
	ln8411->pdata->iin_cfg = ln8411->iin_cc;

	/* Calculate new TA max voltage, current */
	val = ln8411->iin_cc / PD_MSG_TA_CUR_STEP;
	iin_cc = val * PD_MSG_TA_CUR_STEP;

	val = ln8411->ta_max_pwr / (iin_cc / ln8411->chg_mode  / 1000); /* mV */

	/* Adjust values with APDO resolution(20mV) */
	val = val * 1000 / PD_MSG_TA_VOL_STEP;
	val = val * PD_MSG_TA_VOL_STEP; /* uV */
	ln8411->ta_max_vol = min(val, ta_max_vol);

	ln8411->ta_vol = 2 * vbat * ln8411->chg_mode + LN8411_TA_VOL_PRE_OFFSET;

	/* PPS voltage resolution is 20mV */
	val = ln8411->ta_vol / PD_MSG_TA_VOL_STEP;
	ln8411->ta_vol = val * PD_MSG_TA_VOL_STEP;
	ln8411->ta_vol = min(ln8411->ta_vol, ln8411->ta_max_vol);

	ln8411->ta_cur = min((int)ln8411->ta_max_cur, iin_cc + LN8411_TA_CUR_MAX_OFFSET);

	logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
			"%s: iin_cc=%d, ta_vol=%d ta_cur=%d ta_max_vol=%d",
			__func__, ln8411->iin_cc, ln8411->ta_vol, ln8411->ta_cur,
			ln8411->ta_max_vol);

	return 0;
}

/*
 * like ln8411_preset_dcmode() but will not query the TA.
 * Called from timer:
 * [ln8411_charge_ccmode | ln8411_charge_cvmode] ->
 * 	ln8411_apply_new_iin() ->
 * 		ln8411_adjust_ta_current() ->
 * 			ln8411_reset_dcmode()
 * 	ln8411_apply_new_vfloat() ->
 * 		ln8411_reset_dcmode()
 *
 * NOTE: caller holds mutex_lock(&ln8411->lock);
 */
static int ln8411_reset_dcmode(struct ln8411_charger *ln8411)
{
	int ret = -EINVAL, vbat;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);
	dev_dbg(ln8411->dev, "%s: = charging_state=%u == \n", __func__,
		 ln8411->charging_state);

	if (ln8411->cc_max < 0) {
		dev_err(ln8411->dev, "%s: invalid cc_max=%d\n", __func__, ln8411->cc_max);
		goto error;
	}

	/*
	 * VBAT is over threshold but it might be "bouncy" due to transitory
	 * used to determine ta_vout.
	 */
	vbat = ln8411_read_adc(ln8411, ADCCH_VBAT);
	if (vbat < 0)
		return vbat;

	/* Check the TA type and set the charging mode */
	if (ln8411->ta_type == TA_TYPE_WIRELESS)
		ret = ln8411_set_wireless_dc(ln8411, vbat);
	else
		ret = ln8411_set_wired_dc(ln8411, vbat);

	/* Clear previous IIN ADC, TA increment flag */
	ln8411->prev_inc = INC_NONE;
	ln8411->prev_iin = 0;
error:
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The caller was triggered from ln8411_apply_new_iin(), return to the
 * calling CC or CV loop.
 * call holding mutex_unlock(&ln8411->lock);
 */
static void ln8411_return_to_loop(struct ln8411_charger *ln8411)
{
	switch (ln8411->ret_state) {
	case DC_STATE_CC_MODE:
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_CV_MODE:
		ln8411->timer_id = TIMER_CHECK_CVMODE;
		break;
	default:
		dev_err(ln8411->dev, "%s: invalid ret_state=%u\n",
			__func__, ln8411->ret_state);
		return;
	}

	dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
		 ln8411->charging_state, ln8411->ret_state);

	ln8411->charging_state = ln8411->ret_state;
	ln8411->timer_period = 1000;
	ln8411->ret_state = 0;
	ln8411->new_iin = 0;
}

/*
 * Kicked from ln8411_apply_new_iin() when ln8411->new_iin!=0 and completed
 * off the timer. Never called on WLC_DC.
 * NOTE: Will return to the calling loop in ->ret_state
 */
static int ln8411_adjust_ta_current(struct ln8411_charger *ln8411)
{
	const int ta_limit = ln8411->iin_cc;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;
	int ret = 0;

	rc = ln8411_check_error(ln8411);
	if (rc != 0)
		return rc;

	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=%d ta_limit=%d, iin_cfg=%d icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, ln8411->iin_cc, ta_limit, ln8411->pdata->iin_cfg,
			icn, ibat, ln8411->cc_max, rc);

	if (ln8411->charging_state != DC_STATE_ADJUST_TACUR)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_ADJUST_TACUR);

	ln8411->charging_state = DC_STATE_ADJUST_TACUR;

	if (ln8411->ta_cur == ta_limit) {

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"adj. End, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				ln8411->ta_cur, ln8411->ta_vol,
				ln8411->iin_cc, ln8411->chg_mode);

		/* "Recover" IIN_CC to the original value (new_iin) */
		ln8411->iin_cc = ln8411->new_iin;
		ln8411_return_to_loop(ln8411);

	} else if (ln8411->iin_cc > ln8411->pdata->iin_cfg) {
		const int old_iin_cfg = ln8411->pdata->iin_cfg;

		/* Raise iin_cfg to the new iin_cc value (why??!?!?) */
		ln8411->pdata->iin_cfg = ln8411->iin_cc;

		ret = ln8411_set_input_current(ln8411, ln8411->iin_cc);
		if (ret == 0)
			ret = ln8411_reset_dcmode(ln8411);
		if (ret < 0)
			goto error;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"New IIN, ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, iin_cfg=%d->%d chg_mode=%u",
				ln8411->ta_max_vol, ln8411->ta_max_cur,
				ln8411->ta_max_pwr, ln8411->iin_cc,
				old_iin_cfg, ln8411->iin_cc,
				ln8411->chg_mode);

		ln8411->new_iin = 0;

		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_ADJUST_CC);

		/* Send PD Message and go to Adjust CC mode */
		ln8411->charging_state = DC_STATE_ADJUST_CC;
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	} else {
		unsigned int val;

		/*
		 * Adjust IIN_CC with APDO resolution(50mA)
		 * ln8411->iin_cc will be reset to ln8411->new_iin when
		 * ->ta_cur reaches the ta_limit at the beginning of the
		 * function
		 */
		val = ln8411->iin_cc / PD_MSG_TA_CUR_STEP;
		ln8411->iin_cc = val * PD_MSG_TA_CUR_STEP;
		ln8411->ta_cur = ln8411->iin_cc;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "adjust iin=%u ta_cur=%d chg_mode=%d",
				ln8411->iin_cc, ln8411->ta_cur, ln8411->chg_mode);

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	}

	/* reschedule on ret == 0 */
error:
	return ret;
}

/* Kicked from apply_new_iin() then run off the timer
 * call holding mutex_lock(&ln8411->lock);
 */
static int ln8411_adjust_ta_voltage(struct ln8411_charger *ln8411)
{
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (ln8411->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_ADJUST_TAVOL);

	ln8411->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = ln8411_check_error(ln8411);
	if (rc != 0)
		return rc;

	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;

	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, ln8411->iin_cc - PD_MSG_TA_CUR_STEP,
			ln8411->iin_cc, ln8411->iin_cc + PD_MSG_TA_CUR_STEP,
			icn, ibat, ln8411->cc_max, rc);

	if (iin < 0)
		return iin;


	/* Compare IIN ADC with targer input current */
	if (iin > (ln8411->iin_cc + PD_MSG_TA_CUR_STEP)) {
		/* TA current is higher than the target input current */
		/* Decrease TA voltage (20mV) */
		ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1, ta_vol=%u",
				ln8411->ta_vol);

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	} else if (iin < (ln8411->iin_cc - PD_MSG_TA_CUR_STEP)) {
		/* TA current is lower than the target input current */

		if (ln8411_check_status(ln8411) == STS_MODE_VFLT_LOOP) {
			/* IIN current may not able to increase in CV */

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					ln8411->ta_cur, ln8411->ta_vol,
					ln8411->iin_cc, ln8411->chg_mode);

			ln8411_return_to_loop(ln8411);
		} else if (ln8411->ta_vol == ln8411->ta_max_vol) {
			/* TA TA voltage is already at the maximum voltage */

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"End1, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
					ln8411->ta_cur, ln8411->ta_vol,
					ln8411->iin_cc, ln8411->chg_mode);

			ln8411_return_to_loop(ln8411);
		} else {
			/* Increase TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol + PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2, ta_vol=%u",
					ln8411->ta_vol);

			/* Send PD Message */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */
		/* IIN_CC - 50mA < IIN ADC < IIN_CC + 50mA  */

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End2, ta_cur=%u, ta_vol=%u, iin_cc=%u, chg_mode=%u",
				ln8411->ta_cur, ln8411->ta_vol,
			ln8411->iin_cc, ln8411->chg_mode);

		ln8411_return_to_loop(ln8411);
	}

	return 0;
}

/*
 * Kicked from apply_new_iin() then run off the timer
 * * NOTE: caller must hold mutex_lock(&ln8411->lock)
 */
static int ln8411_adjust_rx_voltage(struct ln8411_charger *ln8411)
{
	const int iin_high = ln8411->iin_cc + ln8411->pdata->iin_cc_comp_offset;
	const int iin_low = ln8411->iin_cc - ln8411->pdata->iin_cc_comp_offset;
	int rc, ibat, icn = -EINVAL, iin = -EINVAL;
	bool ovc_flag;

	if (ln8411->charging_state != DC_STATE_ADJUST_TAVOL)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_ADJUST_TAVOL);

	ln8411->charging_state = DC_STATE_ADJUST_TAVOL;

	rc = ln8411_check_error(ln8411);
	if (rc != 0)
		return rc;

	rc = ln8411_get_current_adcs(ln8411, &ibat, &icn, &iin);
	if (rc)
		return rc;


	ovc_flag = ibat > ln8411->cc_max;
	if (ovc_flag)
		ln8411_chg_stats_inc_ovcf(&ln8411->chg_data, ibat, ln8411->cc_max);

	logbuffer_prlog(ln8411, ovc_flag ? LOGLEVEL_WARNING : LOGLEVEL_DEBUG,
			"%s: iin=%d, iin_cc=[%d,%d,%d], icn=%d ibat=%d, cc_max=%d rc=%d",
			__func__, iin, iin_low, ln8411->iin_cc, iin_high,
			icn, ibat, ln8411->cc_max, rc);

	if (iin < 0)
		return iin;

	/* Compare IIN ADC with targer input current */
	if (iin > iin_high) {
		/* RX current is higher than the target input current */

		/* Decrease RX voltage (100mV) */
		ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1, rx_vol=%u",
				ln8411->ta_vol);

		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	} else if (iin < iin_low) {
		/* RX current is lower than the target input current */

		if (ln8411_check_status(ln8411) == STS_MODE_VFLT_LOOP) {
			/* RX current may not able to increase in CV */
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"End1-1, skip adjust for cv, rx_vol=%u, iin_cc=%u",
					ln8411->ta_vol, ln8411->iin_cc);

			ln8411_return_to_loop(ln8411);
		} else if (ln8411->ta_vol == ln8411->ta_max_vol) {
			/* RX current is already the maximum voltage */
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"End1, rx_vol=%u, iin_cc=%u, chg_mode=%u",
					ln8411->ta_vol, ln8411->iin_cc,
					ln8411->chg_mode);

			/* Return charging state to the previous state */
			ln8411_return_to_loop(ln8411);
		} else {
			/* Increase RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol + WCRX_VOL_STEP;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2, rx_vol=%u",
					ln8411->ta_vol);

			/* Set RX voltage */
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	} else {
		/* IIN ADC is in valid range */

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End2, rx_vol=%u, iin_cc=%u, chg_mode=%u",
				ln8411->ta_vol, ln8411->iin_cc,
				ln8411->chg_mode);

		/* Return charging state to the previous state */
		ln8411_return_to_loop(ln8411);
	}

	return 0;
}

/*
 * Called from CC and CV loops to set a new IIN (i.e. a new cc_max charging
 * current). Should also change the iin_cfg to avoid overcurrents.
 * NOTE: caller must hold mutex_lock(&ln8411->lock)
 */
static int ln8411_apply_new_iin(struct ln8411_charger *ln8411)
{
	int ret;

	logbuffer_prlog(ln8411, LOGLEVEL_INFO,
			"new_iin=%d (cc_max=%d), ta_type=%d charging_state=%d",
			ln8411->new_iin, ln8411->cc_max,
			ln8411->ta_type, ln8411->charging_state);

	/* iin_cfg is adjusted UP in ln8411_set_input_current() */
	ret = ln8411_set_input_current(ln8411, ln8411->new_iin);
	if (ret < 0)
		return ret;
	ln8411->pdata->iin_cfg = ln8411->new_iin;

	 /*
	  * ->ret_state is used to go back to the loop (CC or CV) that called
	  * this function.
	  */
	ln8411->ret_state = ln8411->charging_state;

	/*
	 * new_iin is used to trigger the process which might span one or more
	 * timer ticks the new_iin . The flag will be cleared once the target
	 * is reached.
	 */
	ln8411->iin_cc = ln8411->new_iin;
	if (ln8411->ta_type == TA_TYPE_WIRELESS) {
		ret = ln8411_adjust_rx_voltage(ln8411);
	} else if (ln8411->iin_cc < LN8411_TA_MIN_CUR) {
		/* TA current = LN8411_TA_MIN_CUR(1.0A) */
		ln8411->ta_cur = LN8411_TA_MIN_CUR;
		ret = ln8411_adjust_ta_voltage(ln8411);
	} else {
		ret = ln8411_adjust_ta_current(ln8411);
	}

	/* need reschedule on ret != 0 */

	dev_dbg(ln8411->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * also called from ln8411_set_new_cc_max()
 * call holding mutex_unlock(&ln8411->lock);
 */
static int ln8411_set_new_iin(struct ln8411_charger *ln8411, int iin)
{
	int ret = 0;

	if (iin < 0) {
		dev_dbg(ln8411->dev, "%s: ignore negative iin=%d\n", __func__, iin);
		return 0;
	}

	/* same as previous request nevermind */
	if (iin == ln8411->new_iin)
		return 0;

	dev_dbg(ln8411->dev, "%s: new_iin=%d->%d state=%d\n", __func__,
		 ln8411->new_iin, iin, ln8411->charging_state);

	/* apply iin_cc in ln8411_preset_config() at start */
	if (ln8411->charging_state == DC_STATE_NO_CHARGING ||
	    ln8411->charging_state == DC_STATE_CHECK_VBAT) {

		/* used on start vs the ->iin_cfg one */
		ln8411->pdata->iin_cfg = iin;
		ln8411->iin_cc = iin;
	} else if (ln8411->ret_state == 0) {
		/*
		 * ln8411_apply_new_iin() has not picked out the value yet
		 * and the value can be changed safely.
		 */
		ln8411->new_iin = iin;

		/* might want to tickle the loop now */
	} else {
		/* the caller must retry */
		ret = -EAGAIN;
	}

	dev_dbg(ln8411->dev, "%s: ret=%d\n", __func__, ret);
	return ret;
}

/*
 * The is no CC loop in this part: current must be controlled on TA side
 * adjusting output power. cc_max (the charging current) is scaled to iin
 *
 */
static int ln8411_set_new_cc_max(struct ln8411_charger *ln8411, int cc_max)
{
	const int prev_cc_max = ln8411->cc_max;
	int iin_max, ret = 0;

	if (cc_max < 0) {
		dev_dbg(ln8411->dev, "%s: ignore negative cc_max=%d\n", __func__, cc_max);
		return 0;
	}

	mutex_lock(&ln8411->lock);

	/* same as previous request nevermind */
	if (cc_max == ln8411->cc_max)
		goto done;

	/* iin will be capped by the adapter capabilities in reset_dcmode() */
	iin_max = ln8411_get_iin_max(ln8411, cc_max);
	if (iin_max <= 0) {
		dev_dbg(ln8411->dev, "%s: ignore negative iin_max=%d\n", __func__, iin_max);
		goto done;
	}

	ret = ln8411_set_new_iin(ln8411, iin_max);
	if (ret == 0)
		ln8411->cc_max = cc_max;

	logbuffer_prlog(ln8411, LOGLEVEL_INFO,
			"%s: charging_state=%d cc_max=%d->%d iin_max=%d, ret=%d",
			__func__, ln8411->charging_state, prev_cc_max,
			cc_max, iin_max, ret);

done:
	dev_dbg(ln8411->dev, "%s: ret=%d\n", __func__, ret);
	mutex_unlock(&ln8411->lock);
	return ret;
}

/*
 * Apply ln8411->new_vfloat to the charging voltage.
 * Called from CC and CV loops, needs mutex_lock(&ln8411->lock)
 */
static int ln8411_apply_new_vfloat(struct ln8411_charger *ln8411)
{
	int ret = 0;

	if (ln8411->fv_uv == ln8411->new_vfloat)
		goto error_done;

	/* actually change the hardware */
	ret = ln8411_set_vfloat(ln8411, ln8411->new_vfloat);
	if (ret < 0)
		goto error_done;

	/* Restart the process if tier switch happened (either direction) */
	if (ln8411->charging_state == DC_STATE_CV_MODE
	    && abs(ln8411->new_vfloat - ln8411->fv_uv) > LN8411_TIER_SWITCH_DELTA) {
		ret = ln8411_reset_dcmode(ln8411);
		if (ret < 0) {
			pr_err("%s: cannot reset dcmode (%d)\n", __func__, ret);
		} else {
			dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
				ln8411->charging_state, DC_STATE_ADJUST_CC);

			ln8411->charging_state = DC_STATE_ADJUST_CC;
			ln8411->timer_id = TIMER_PDMSG_SEND;
			ln8411->timer_period = 0;
		}
	}

	ln8411->fv_uv = ln8411->new_vfloat;

error_done:
	logbuffer_prlog(ln8411, LOGLEVEL_INFO,
			"%s: new_vfloat=%d, ret=%d", __func__,
			ln8411->new_vfloat, ret);

	if (ret == 0)
		ln8411->new_vfloat = 0;

	return ret;
}

static int ln8411_set_new_vfloat(struct ln8411_charger *ln8411, int vfloat)
{
	int ret = 0;

	if (vfloat < 0) {
		dev_dbg(ln8411->dev, "%s: ignore negative vfloat %d\n", __func__, vfloat);
		return 0;
	}

	mutex_lock(&ln8411->lock);
	if (ln8411->new_vfloat == vfloat)
		goto done;

	/* use fv_uv at start in ln8411_preset_config() */
	if (ln8411->charging_state == DC_STATE_NO_CHARGING ||
	    ln8411->charging_state == DC_STATE_CHECK_VBAT) {
		ln8411->fv_uv = vfloat;
	} else {
		/* applied in ln8411_apply_new_vfloat() from CC or in CV loop */
		ln8411->new_vfloat = vfloat;
		pr_debug("%s: new_vfloat=%d\n", __func__, ln8411->new_vfloat);

		/* might want to tickle the cycle */
	}

done:
	mutex_unlock(&ln8411->lock);
	return ret;
}

/* called on loop inactive */
static void ln8411_adjust_ccmode_wireless(struct ln8411_charger *ln8411, int iin)
{
	/* IIN_ADC > IIN_CC -20mA ? */
	if (iin > (ln8411->iin_cc - LN8411_IIN_ADC_OFFSET)) {
		/* Input current is already over IIN_CC */
		/* End RX voltage adjustment */

		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		ln8411->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End1: IIN_ADC=%d, rx_vol=%u",
				iin, ln8411->ta_vol);

		/* Clear TA increment flag */
		ln8411->prev_inc = INC_NONE;
		/* Go to CC mode */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 0;

	/* Check RX voltage */
	} else if (ln8411->ta_vol == ln8411->ta_max_vol) {
		/* RX voltage is already max value */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,"End2: MAX value, rx_vol=%u max=%d",
				ln8411->ta_vol, ln8411->ta_max_vol);

		/* Clear TA increment flag */
		ln8411->prev_inc = INC_NONE;
		/* Go to CC mode */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 0;
	} else {
		/* Try to increase RX voltage(100mV) */
		ln8411->ta_vol = ln8411->ta_vol + WCRX_VOL_STEP;
		if (ln8411->ta_vol > ln8411->ta_max_vol)
			ln8411->ta_vol = ln8411->ta_max_vol;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont: rx_vol=%u",
				ln8411->ta_vol);
		/* Set RX voltage */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	}
}

/* called on loop inactive */
static void ln8411_adjust_ccmode_wired(struct ln8411_charger *ln8411, int iin)
{

	/* USBPD TA is connected */
	if (iin > (ln8411->iin_cc - LN8411_IIN_ADC_OFFSET)) {
		/* IIN_ADC > IIN_CC -20mA ? */
		/* Input current is already over IIN_CC */
		/* End TA voltage and current adjustment */

		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CC_MODE);

		/* change charging state to CC mode */
		ln8411->charging_state = DC_STATE_CC_MODE;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End1: IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, ln8411->ta_vol, ln8411->ta_cur);

		/* Clear TA increment flag */
		ln8411->prev_inc = INC_NONE;
		/* Go to CC mode */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 0;

	/* Check TA voltage */
	} else if (ln8411->ta_vol == ln8411->ta_max_vol) {
		/* TA voltage is already max value */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End2: MAX value, ta_vol=%u, ta_cur=%u",
				ln8411->ta_vol, ln8411->ta_cur);

		/* Clear TA increment flag */
		ln8411->prev_inc = INC_NONE;
		/* Go to CC mode */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 0;

		/* Check TA tolerance
		 * The current input current compares the final input
		 * current(IIN_CC) with 100mA offset PPS current tolerance
		 * has +/-150mA, so offset defined 100mA(tolerance +50mA)
		 */
	} else if (iin < (ln8411->iin_cc - LN8411_TA_IIN_OFFSET)) {
		/*
		 * TA voltage too low to enter TA CC mode, so we
		 * should increase TA voltage
		 */
		ln8411->ta_vol = ln8411->ta_vol + LN8411_TA_VOL_STEP_ADJ_CC *
					ln8411->chg_mode;

		if (ln8411->ta_vol > ln8411->ta_max_vol)
			ln8411->ta_vol = ln8411->ta_max_vol;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont1: ta_vol=%u",
				ln8411->ta_vol);

		/* Set TA increment flag */
		ln8411->prev_inc = INC_TA_VOL;
		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

	/* compare IIN ADC with previous IIN ADC + 20mA */
	} else if (iin > (ln8411->prev_iin + LN8411_IIN_ADC_OFFSET)) {
		/* TA can supply more current if TA voltage is high */
		/* TA voltage too low for TA CC mode: increase it */
		ln8411->ta_vol = ln8411->ta_vol +
					LN8411_TA_VOL_STEP_ADJ_CC *
					ln8411->chg_mode;
		if (ln8411->ta_vol > ln8411->ta_max_vol)
			ln8411->ta_vol = ln8411->ta_max_vol;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont2: ta_vol=%u",
				ln8411->ta_vol);
		/* Set TA increment flag */
		ln8411->prev_inc = INC_TA_VOL;

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

	/* Check the previous increment */
	} else if (ln8411->prev_inc == INC_TA_CUR) {
		/*
		 * The previous increment is TA current, but input
		 * current does not increase. Try with voltage.
		 */

		ln8411->ta_vol = ln8411->ta_vol +
					LN8411_TA_VOL_STEP_ADJ_CC *
					ln8411->chg_mode;
		if (ln8411->ta_vol > ln8411->ta_max_vol)
			ln8411->ta_vol = ln8411->ta_max_vol;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont3: ta_vol=%u",
				ln8411->ta_vol);

		ln8411->prev_inc = INC_TA_VOL;
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

		/*
		 * The previous increment is TA voltage, but input
		 * current does not increase
		 */

		/* Try to increase TA current */
		/* Check APDO max current */
	} else if (!ln8411_can_inc_ta_cur(ln8411)) {
		/* TA current is maximum current */

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"End(MAX_CUR): IIN_ADC=%d, ta_vol=%u, ta_cur=%u",
				iin, ln8411->ta_vol, ln8411->ta_cur);

		ln8411->prev_inc = INC_NONE;

		/* Go to CC mode */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 0;
	} else {
		/* TA has tolerance and compensate it as real current */
		/* Increase TA current(50mA) */
		ln8411->ta_cur = ln8411->ta_cur + PD_MSG_TA_CUR_STEP;
		if (ln8411->ta_cur > ln8411->ta_max_cur)
			ln8411->ta_cur = ln8411->ta_max_cur;

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "Cont4: ta_cur=%u",
				ln8411->ta_cur);

		ln8411->prev_inc = INC_TA_CUR;
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
	}
}

static int ln8411_vote_dc_avail(struct ln8411_charger *ln8411, int vote, int enable)
{
	int ret = 0;

	if (!ln8411->dc_avail)
		ln8411->dc_avail = gvotable_election_get_handle(VOTABLE_DC_CHG_AVAIL);

	if (ln8411->dc_avail) {
		ret = gvotable_cast_int_vote(ln8411->dc_avail, REASON_DC_DRV, vote, enable);
		if (ret < 0)
			dev_err(ln8411->dev, "Unable to cast vote for DC Chg avail (%d)\n", ret);
	}

	logbuffer_prlog(ln8411, ln8411->charging_state == DC_STATE_ERROR ?
			LOGLEVEL_INFO : LOGLEVEL_DEBUG,
			"%s: Voting dc_avail when in error state", __func__);

	return ret;
}

/* Direct Charging Adjust CC MODE control
 * called at the beginnig of CC mode charging. Will be followed by
 * ln8411_charge_ccmode with which share some of the adjustments.
 */
static int ln8411_charge_adjust_ccmode(struct ln8411_charger *ln8411)
{
	int  iin, ccmode, vbatt, vin_vol;
	int ret = 0;

	mutex_lock(&ln8411->lock);

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);
	ln8411_prlog_state(ln8411, __func__);

	if (ln8411->charging_state != DC_STATE_ADJUST_CC)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_ADJUST_CC);

	ln8411->charging_state = DC_STATE_ADJUST_CC;

	ret = ln8411_check_error(ln8411);
	if (ret != 0)
		goto error; /*This is not active mode. */

	ccmode = ln8411_check_status(ln8411);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}

	switch(ccmode) {
	case STS_MODE_IIN_LOOP:
		fallthrough;
	case STS_MODE_CHG_LOOP:	/* CHG_LOOP does't exist */
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End1: rx_vol=%u",
					 ln8411->ta_vol);
		} else if (ln8411->ta_cur > LN8411_TA_MIN_CUR) {
			/* TA current is higher than 1.0A */
			/* Decrease TA current (50mA) */
			ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End2: ta_cur=%u, ta_vol=%u",
					ln8411->ta_cur, ln8411->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End3: ta_cur=%u, ta_vol=%u",
					ln8411->ta_cur, ln8411->ta_vol);
		}

		ln8411->prev_inc = INC_NONE;

		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CC_MODE);

		/* Send PD Message and then go to CC mode */
		ln8411->charging_state = DC_STATE_CC_MODE;
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		vbatt = ln8411_read_adc(ln8411, ADCCH_VBAT);

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "End4: vbatt=%d, ta_vol=%u",
				vbatt, ln8411->ta_vol);

		/* Clear TA increment flag */
		ln8411->prev_inc = INC_NONE;
		/* Go to Pre-CV mode */
		ln8411->timer_id = TIMER_ENTER_CVMODE;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:

		iin = ln8411_read_adc(ln8411, ADCCH_IIN);
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"Inactive: iin=%d, iin_cc=%d, cc_max=%d",
				iin, ln8411->iin_cc, ln8411->cc_max);
		if (iin < 0)
			break;

		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			ln8411_adjust_ccmode_wireless(ln8411, iin);
		} else {
			ln8411_adjust_ccmode_wired(ln8411, iin);
		}

		ln8411->prev_iin = iin;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = ln8411_read_adc(ln8411, ADCCH_VIN);

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "VIN_UVLO: ta_vol=%u, vin_vol=%d",
				ln8411->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		ln8411->timer_id = TIMER_ADJUST_CCMODE;
		ln8411->timer_period = 1000;
		break;

	default:
		goto error;
	}

	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 msecs_to_jiffies(ln8411->timer_period));
error:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* <0 error, 0 no new limits, >0 new limits */
static int ln8411_apply_new_limits(struct ln8411_charger *ln8411)
{
	int ret = 0;

	if (ln8411->new_iin && ln8411->new_iin < ln8411->iin_cc) {
		ret = ln8411_apply_new_iin(ln8411);
		if (ret == 0)
			ret = 1;
	} else if (ln8411->new_vfloat) {
		ret = ln8411_apply_new_vfloat(ln8411);
		if (ret == 0)
			ret = 1;
	} else if (ln8411->new_iin) {
		ret = ln8411_apply_new_iin(ln8411);
		if (ret == 0)
			ret = 1;
	} else {
		return 0;
	}

	return ret;
}

/* 2:1 Direct Charging CC MODE control */
static int ln8411_charge_ccmode(struct ln8411_charger *ln8411)
{
	int ccmode = -1, vin_vol, iin, ret = 0;

	dev_dbg(ln8411->dev, "%s: ======START======= \n", __func__);

	mutex_lock(&ln8411->lock);

	if (ln8411->charging_state != DC_STATE_CC_MODE)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CC_MODE);

	ln8411->charging_state = DC_STATE_CC_MODE;

	ln8411_prlog_state(ln8411, __func__);

	ret = ln8411_check_error(ln8411);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in VFLOAT here means that we have busted the tier, a
	 * change in iin means that the thermal engine had changed cc_max.
	 * ln8411_apply_new_limits() changes ln8411->charging_state to
	 * DC_STATE_ADJUST_TAVOL or DC_STATE_ADJUST_TACUR when new limits
	 * need to be applied.
	 */
	ret = ln8411_apply_new_limits(ln8411);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	ccmode = ln8411_check_status(ln8411);
	if (ccmode < 0) {
		ret = ccmode;
		goto error_exit;
	}

	switch(ccmode) {
	case STS_MODE_LOOP_INACTIVE:

		/* Set input current compensation */
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Need RX voltage compensation */
			ret = ln8411_set_rx_voltage_comp(ln8411);

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "INACTIVE1: rx_vol=%u",
					ln8411->ta_vol);
		} else {
			const int ta_max_vol = ln8411->ta_max_vol;
			int ta_max_vol_cp = 0;

			if (ln8411->chg_mode == CHG_4TO1_DC_MODE)
				ta_max_vol_cp = ln8411->pdata->ta_max_vol_4_1 * CHG_4TO1_DC_MODE;
			else if (ln8411->chg_mode == CHG_2TO1_DC_MODE)
				ta_max_vol_cp = ln8411->pdata->ta_max_vol_2_1 * CHG_2TO1_DC_MODE;

			/* Check TA current with TA_MIN_CUR */
			if (ln8411->ta_cur <= LN8411_TA_MIN_CUR) {
				ln8411->ta_cur = LN8411_TA_MIN_CUR;

				ret = ln8411_set_ta_voltage_comp(ln8411);
			} else if (ta_max_vol >= ta_max_vol_cp) {
				ret = ln8411_set_ta_current_comp(ln8411);
			} else {
				/* constant power mode */
				ret = ln8411_set_ta_current_comp2(ln8411);
			}

			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"INACTIVE2: ta_cur=%u, ta_vol=%u",
					ln8411->ta_cur,
					ln8411->ta_vol);
		}
		break;

	case STS_MODE_VFLT_LOOP:
		/* TODO: adjust fv_uv here based on real vbatt */

		iin = ln8411_read_adc(ln8411, ADCCH_IIN);
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG, "CC VFLOAT: iin=%d", iin);

		/* go to Pre-CV mode */
		ln8411->timer_id = TIMER_ENTER_CVMODE;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_IIN_LOOP:
		fallthrough;
	case STS_MODE_CHG_LOOP:
		iin = ln8411_read_adc(ln8411, ADCCH_IIN);
		if (iin < 0)
			break;

		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"IIN_LOOP1: iin=%d, next_rx_vol=%u",
					iin, ln8411->ta_vol);
		} else if (ln8411->ta_cur <= LN8411_TA_MIN_CUR) {
			/* Decrease TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"IIN_LOOP2: iin=%d, next_ta_vol=%u",
					iin, ln8411->ta_vol);
		} else {
			/* Decrease TA current (50mA) */
			ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"IIN_LOOP3: iin=%d, next_ta_cur=%u",
					iin, ln8411->ta_cur);
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = ln8411_read_adc(ln8411, ADCCH_VIN);

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				ln8411->ta_cur, ln8411->ta_vol, vin_vol);

		/* Check VIN after 1sec */
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		ln8411->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 msecs_to_jiffies(ln8411->timer_period));

error_exit:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ccmode=%d timer_id=%d, timer_period=%lu ret=%d\n",
		 __func__, ccmode, ln8411->timer_id, ln8411->timer_period,
		 ret);
	return ret;
}


/* Direct Charging Start CV MODE control - Pre CV MODE */
static int ln8411_charge_start_cvmode(struct ln8411_charger *ln8411)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&ln8411->lock);

	if (ln8411->charging_state != DC_STATE_START_CV)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_START_CV);

	ln8411->charging_state = DC_STATE_START_CV;

	/* Check the charging type */
	ret = ln8411_check_error(ln8411);
	if (ret != 0)
		goto error_exit;

	/* Check the status */
	cvmode = ln8411_check_status(ln8411);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	switch(cvmode) {
	case STS_MODE_CHG_LOOP:
		fallthrough;
	case STS_MODE_IIN_LOOP:

		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: PreCV IIN_LOOP: rx_vol=%u",
				 __func__, ln8411->ta_vol);
		} else {
			/* Check TA current */
			if (ln8411->ta_cur > LN8411_TA_MIN_CUR) {
				/* TA current is higher than 1.0A */

				/* Decrease TA current (50mA) */
				ln8411->ta_cur = ln8411->ta_cur - PD_MSG_TA_CUR_STEP;
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_cur=%u",
						__func__, ln8411->ta_cur);
			} else {
				/* TA current is less than 1.0A */
				/* Decrease TA voltage (20mV) */
				ln8411->ta_vol = ln8411->ta_vol - PD_MSG_TA_VOL_STEP;
				logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
						"%s: PreCV IIN_LOOP: ta_vol=%u",
						__func__, ln8411->ta_vol);
			}
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: rx_vol=%u",
					__func__, ln8411->ta_vol);
		} else {
			/* Decrease TA voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol -
					  LN8411_TA_VOL_STEP_PRE_CV *
					  ln8411->chg_mode;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: PreCV VF Cont: ta_vol=%u",
					__func__, ln8411->ta_vol);
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: PreCV End: ta_vol=%u, ta_cur=%u",
				__func__, ln8411->ta_vol, ln8411->ta_cur);

		/* Need to implement notification to other driver */
		/* To do here */

		/* Go to CV mode */
		ln8411->timer_id = TIMER_CHECK_CVMODE;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification , it works by hardware */
		vin_vol = ln8411_read_adc(ln8411, ADCCH_VIN);

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: PreCV VIN_UVLO: ta_vol=%u, vin_vol=%u",
				__func__, ln8411->ta_cur, vin_vol);

		/* Check VIN after 1sec */
		ln8411->timer_id = TIMER_ENTER_CVMODE;
		ln8411->timer_period = 1000;
		break;

	default:
		break;
	}

	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 msecs_to_jiffies(ln8411->timer_period));
error_exit:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int ln8411_check_eoc(struct ln8411_charger *ln8411)
{
	const int eoc_tolerance = 25000; /* 25mV under max float voltage */
	const int vlimit = LN8411_COMP_VFLOAT_MAX - eoc_tolerance;
	int iin, vbat;

	iin = ln8411_read_adc(ln8411, ADCCH_IIN);
	if (iin < 0) {
		dev_err(ln8411->dev, "%s: iin=%d\n", __func__, iin);
		return iin;
	}

	vbat = ln8411_read_adc(ln8411, ADCCH_VBAT);
	if (vbat < 0) {
		dev_err(ln8411->dev, "%s: vbat=%d\n", __func__, vbat);
		return vbat;
	}

	dev_dbg(ln8411->dev, "%s: iin=%d, topoff=%u, vbat=%d vlimit=%d\n", __func__,
		 iin, ln8411->pdata->iin_topoff,
		 vbat, vlimit);

	return iin < ln8411->pdata->iin_topoff && vbat >= vlimit;
}

/*  Direct Charging CV MODE control */
static int ln8411_charge_cvmode(struct ln8411_charger *ln8411)
{
	int ret = 0;
	int cvmode;
	int vin_vol;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&ln8411->lock);

	if (ln8411->charging_state != DC_STATE_CV_MODE)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CV_MODE);

	ln8411->charging_state = DC_STATE_CV_MODE;

	ret = ln8411_check_error(ln8411);
	if (ret != 0)
		goto error_exit;

	/*
	 * A change in vfloat and cc_max here is a normal tier transition, a
	 * change in iin  means that the thermal engine has changed cc_max.
	 */
	ret = ln8411_apply_new_limits(ln8411);
	if (ret < 0)
		goto error_exit;
	if (ret > 0)
		goto done;

	cvmode = ln8411_check_status(ln8411);
	if (cvmode < 0) {
		ret = cvmode;
		goto error_exit;
	}

	if (cvmode == STS_MODE_LOOP_INACTIVE) {
		ret = ln8411_check_eoc(ln8411);
		if (ret < 0)
			goto error_exit;
		if (ret)
			cvmode = STS_MODE_CHG_DONE;
	}

	switch(cvmode) {
	case STS_MODE_CHG_DONE: {
		const bool done_already = ln8411->charging_state ==
					  DC_STATE_CHARGING_DONE;

		if (!done_already)
			dev_info(ln8411->dev, "%s: charging_state=%u->%u\n",
				 __func__, ln8411->charging_state,
				 DC_STATE_CHARGING_DONE);


		/* Keep CV mode until driver send stop charging */
		ln8411->charging_state = DC_STATE_CHARGING_DONE;
		power_supply_changed(ln8411->mains);

		/* _cpm already came in */
		if (ln8411->charging_state == DC_STATE_NO_CHARGING) {
			dev_dbg(ln8411->dev, "%s: Already stop DC\n", __func__);
			break;
		}

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: done_already=%d charge Done\n", __func__,
				done_already);

		ln8411->timer_id = TIMER_CHECK_CVMODE;
		ln8411->timer_period = LN8411_CVMODE_CHECK_T;
	} break;

	case STS_MODE_CHG_LOOP:
		fallthrough;
	case STS_MODE_IIN_LOOP:
		/* Check the TA type */
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX Voltage (100mV) */
			ln8411->ta_vol = ln8411->ta_vol -
						WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: rx_vol=%u",
					__func__, ln8411->ta_vol);

		/* Check TA current */
		} else if (ln8411->ta_cur > LN8411_TA_MIN_CUR) {
			/* TA current is higher than (1.0A*chg_mode) */
			/* Decrease TA current (50mA) */
			ln8411->ta_cur = ln8411->ta_cur -
						PD_MSG_TA_CUR_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_cur=%u",
					__func__, ln8411->ta_cur);
		} else {
			/* TA current is less than (1.0A*chg_mode) */
			/* Decrease TA Voltage (20mV) */
			ln8411->ta_vol = ln8411->ta_vol -
						PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: CV LOOP, Cont: ta_vol=%u",
					__func__, ln8411->ta_vol);
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_VFLT_LOOP:
		/* Check the TA type */
		if (ln8411->ta_type == TA_TYPE_WIRELESS) {
			/* Decrease RX voltage */
			ln8411->ta_vol = ln8411->ta_vol - WCRX_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: rx_vol=%u",
					__func__, ln8411->ta_vol);
		} else {
			/* Decrease TA voltage */
			ln8411->ta_vol = ln8411->ta_vol - 2 * PD_MSG_TA_VOL_STEP;
			logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
					"%s: CV VFLOAT, Cont: ta_vol=%u",
					__func__, ln8411->ta_vol);
		}

		/* Send PD Message */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;
		break;

	case STS_MODE_LOOP_INACTIVE:
		ln8411->timer_id = TIMER_CHECK_CVMODE;
		ln8411->timer_period = LN8411_CVMODE_CHECK_T;
		break;

	case STS_MODE_VIN_UVLO:
		/* VIN UVLO - just notification, it works by hardware */
		vin_vol = ln8411_read_adc(ln8411, ADCCH_VIN);
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: CC VIN_UVLO: ta_cur=%u ta_vol=%u, vin_vol=%d",
				__func__, ln8411->ta_cur, ln8411->ta_vol,
				vin_vol);

		/* Check VIN after 1sec */
		ln8411->timer_id = TIMER_CHECK_CVMODE;
		ln8411->timer_period = 1000;
		break;

	default:
		break;
	}

done:
	dev_dbg(ln8411->dev, "%s: reschedule next id=%d period=%ld chg_state=%d\n",
		 __func__, ln8411->timer_id, ln8411->timer_period,
		ln8411->charging_state);

	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 msecs_to_jiffies(ln8411->timer_period));
error_exit:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ret=%d next\n", __func__, ret);
	return ret;
}

static int ln8411_set_chg_mode_by_apdo(struct ln8411_charger *ln8411)
{
	int ret;

	/*
	 * Get the APDO max and set chg_mode.
	 * Returns ->ta_max_vol, ->ta_max_cur, ->ta_max_pwr and
	 * ->ta_objpos for the given ta_max_vol and ta_max_cur.
	 */
	ret = ln8411_get_apdo_max_power(ln8411, ln8411->pdata->ta_max_vol_4_1 * CHG_4TO1_DC_MODE, 0);
	if (ret == 0) {
		ln8411->chg_mode = CHG_4TO1_DC_MODE;
		goto done;
	}

	dev_warn(ln8411->dev, "%s: No APDO to support 4:1 for %d, max_voltage: %d\n",
		 __func__, LN8411_TA_MAX_CUR_4_1, ln8411->pdata->ta_max_vol_4_1 * CHG_4TO1_DC_MODE);

	ret = ln8411_get_apdo_max_power(ln8411, ln8411->pdata->ta_max_vol_2_1 * CHG_2TO1_DC_MODE,
				      LN8411_TA_MAX_CUR);
	if (ret == 0) {
		ln8411->chg_mode = CHG_2TO1_DC_MODE;
		goto done;
	}

	 dev_warn(ln8411->dev, "%s: No APDO to support 2:1 for %d, max_voltage: %d\n",
		  __func__, LN8411_TA_MAX_CUR, ln8411->pdata->ta_max_vol_2_1 * CHG_2TO1_DC_MODE);

	ret = ln8411_get_apdo_max_power(ln8411, ln8411->pdata->ta_max_vol_2_1 * CHG_2TO1_DC_MODE, 0);
	if (ret == 0) {
		ln8411->chg_mode = CHG_2TO1_DC_MODE;
		goto done;
	}

	dev_err(ln8411->dev, "%s: No APDO to support 2:1\n", __func__);
	ln8411->chg_mode = CHG_NO_DC_MODE;
	ln8411->ta_max_vol = 0;
	ln8411->error = LN8411_ERROR_APDO;

done:
	return ret;
}

/*
 * Preset TA voltage and current for Direct Charging Mode using
 * the configured cc_max and fv_uv limits. Used only on start
 */
static int ln8411_preset_dcmode(struct ln8411_charger *ln8411)
{
	int vbat;
	int ret = 0, val;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);
	dev_dbg(ln8411->dev, "%s: = charging_state=%u == \n", __func__,
		 ln8411->charging_state);

	/* gcpm set ->cc_max and ->fv_uv before starting */
	if (ln8411->cc_max < 0 || ln8411->fv_uv < 0) {
		dev_err(ln8411->dev, "%s: cc_max=%d fv_uv=%d invalid\n", __func__,
		       ln8411->cc_max, ln8411->fv_uv);
		return -EINVAL;
	}

	if (ln8411->charging_state != DC_STATE_PRESET_DC)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_PRESET_DC);

	ln8411->charging_state = DC_STATE_PRESET_DC;

	/* VBAT is over threshold but it might be "bouncy" due to transitory */
	vbat = ln8411_read_adc(ln8411, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	}

	/* v_float is set on start from GCPM */
	if (vbat > ln8411->fv_uv) {
		dev_err(ln8411->dev, "%s: vbat adc=%d is higher than VFLOAT=%d\n", __func__,
			vbat, ln8411->fv_uv);
		ret = -EINVAL;
		goto error;
	}

	/* determined by ->cfg_iin and cc_max */
	ln8411->ta_max_cur = ln8411_get_iin_max(ln8411, ln8411->cc_max);
	dev_dbg(ln8411->dev, "%s: ta_max_cur=%u, iin_cfg=%u, ln8411->ta_type=%d\n",
		 __func__, ln8411->ta_max_cur, ln8411->pdata->iin_cfg,
		 ln8411->ta_type);

	/* Integration guide V1.0 Section 5.1 */
	/* validity check before start */
	ret = regmap_read(ln8411->regmap, LN8411_CTRL1, &val);
	if (ret || val) {
		dev_info(ln8411->dev, "%s: validity check LN8411_CTRL1 failed\n", __func__);
		ret = -EAGAIN;
		goto error;
	}

	ret = regmap_read(ln8411->regmap, LN8411_ADC_CTRL, &val);
#ifdef POLL_ADC
	if (ret || !(val & LN8411_ADC_EN)) {
#else
	if (ret || !(val & (LN8411_ADC_EN | LN8411_ADC_DONE_MASK))) {
#endif
		dev_info(ln8411->dev, "%s: validity check LN8411_ADC_CTRL failed\n", __func__);
		ret = -EAGAIN;
		goto error;
	}

	/* Check the TA type and set the charging mode */
	if (ln8411->ta_type == TA_TYPE_WIRELESS) {
		if (ln8411->ftm_mode)
			goto error;
		/* Integration guide V1.0 Section 5.2 */
		/* Set work mode */
		ret = ln8411_set_mode(ln8411, ln8411->chg_mode);
		if (ret)
			goto error;

		ret = ln8411_set_prot_by_chg_mode(ln8411);
		if (ret)
			goto error;

		ret = regmap_write(ln8411->regmap, LN8411_CTRL1, LN8411_WPCGATE_EN);
		if (ret)
			goto error;
		/*
		 * Set the RX max voltage to enough high value to find RX
		 * maximum voltage initially
		 */
		ln8411->ta_max_vol = LN8411_WCRX_MAX_VOL * ln8411->chg_mode;

		/* Get the RX max current/voltage(RX_MAX_CUR/VOL) */
		ret = ln8411_get_rx_max_power(ln8411);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: no RX voltage to support 4:1 (%d)\n",
				__func__, ret);
			ln8411->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		ret = ln8411_set_wireless_dc(ln8411, vbat);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: set wired failed (%d)\n", __func__, ret);
			ln8411->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(ln8411, LOGLEVEL_INFO,
				"Preset DC, rx_max_vol=%u, rx_max_cur=%u, rx_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				ln8411->ta_max_vol, ln8411->ta_max_cur, ln8411->ta_max_pwr,
				ln8411->iin_cc, ln8411->chg_mode);
	} else {
		if (!ln8411->ftm_mode)
			ret = ln8411_set_chg_mode_by_apdo(ln8411);
		if (ret < 0) {
			ln8411->charging_state = DC_STATE_ERROR;
			ln8411_vote_dc_avail(ln8411, 0, 1);
			goto error;
		}

		/* Integration guide V1.0 Section 5.2 */
		/* Set work mode */
		ret = ln8411_set_mode(ln8411, ln8411->chg_mode);
		if (ret)
			goto error;

		ret = ln8411_set_prot_by_chg_mode(ln8411);
		if (ret)
			goto error;

		ret = regmap_write(ln8411->regmap, LN8411_CTRL1, LN8411_OVPGATE_EN);
		if (ret)
			goto error;
		/*
		 * ->ta_max_cur is too high for startup, needs to target
		 * CC before hitting max current AND work to ta_max_cur
		 * from there.
		 */
		ret = ln8411_set_wired_dc(ln8411, vbat);
		if (ret < 0) {
			dev_err(ln8411->dev, "%s: set wired failed (%d)\n", __func__, ret);
			ln8411->chg_mode = CHG_NO_DC_MODE;
			goto error;
		}

		logbuffer_prlog(ln8411, LOGLEVEL_INFO,
				"Preset DC, objpos=%d ta_max_vol=%u, ta_max_cur=%u, ta_max_pwr=%lu, iin_cc=%u, chg_mode=%u",
				ln8411->ta_objpos, ln8411->ta_max_vol, ln8411->ta_max_cur,
				ln8411->ta_max_pwr, ln8411->iin_cc, ln8411->chg_mode);

	}

error:
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* Preset direct charging configuration and start charging */
static int ln8411_preset_config(struct ln8411_charger *ln8411)
{
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);

	mutex_lock(&ln8411->lock);

	dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			ln8411->charging_state, DC_STATE_PRESET_DC);

	ln8411->charging_state = DC_STATE_PRESET_DC;

	/* ->iin_cc and ->fv_uv are configured externally */
	ret = ln8411_set_input_current(ln8411, ln8411->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	ret = ln8411_set_vfloat(ln8411, ln8411->fv_uv);
	if (ret < 0)
		goto error;

	/* Enable LN8411 unless aready enabled */
	ret = ln8411_set_charging(ln8411, true);
	if (ret < 0)
		goto error;

	/* Clear previous iin adc */
	ln8411->prev_iin = 0;
	ln8411->prev_inc = INC_NONE;

	/* Go to CHECK_ACTIVE state after 150ms, 300ms for wireless */
	ln8411->timer_id = TIMER_CHECK_ACTIVE;
	if (ln8411->ta_type == TA_TYPE_WIRELESS)
		ln8411->timer_period = LN8411_ENABLE_WLC_DELAY_T;
	else
		ln8411->timer_period = LN8411_ENABLE_DELAY_T;
	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			   msecs_to_jiffies(ln8411->timer_period));
error:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * Check the charging status at start before entering the adjust cc mode or
 * from ln8411_send_message() after a failure.
 */
static int ln8411_check_active_state(struct ln8411_charger *ln8411)
{
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: ======START=======\n", __func__);
	dev_dbg(ln8411->dev, "%s: = charging_state=%u == \n", __func__,
		 ln8411->charging_state);

	mutex_lock(&ln8411->lock);

	if (ln8411->charging_state != DC_STATE_CHECK_ACTIVE)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CHECK_ACTIVE);

	ln8411->charging_state = DC_STATE_CHECK_ACTIVE;

	ret = ln8411_check_error(ln8411);
	if (ret == 0) {
		/* LN8411 is active state */
		ln8411->retry_cnt = 0;
		ln8411->timer_id = TIMER_ADJUST_CCMODE;
		ln8411->timer_period = 0;
	}

	if (ret) {
		int ret1 = dump_all_regs(ln8411);

		if (ret1)
			dev_err(ln8411->dev, "%s: Error dumping regs (%d)\n", __func__, ret1);
	}

	/* Implement error handler function if it is needed */
	if (ret < 0) {
		logbuffer_prlog(ln8411, LOGLEVEL_ERR,
				"%s: charging_state=%d, not active or error (%d)",
				__func__, ln8411->charging_state, ret);
		if (ret != -EAGAIN) {
			ln8411->timer_id = TIMER_ID_NONE;
			dev_err(ln8411->dev, "%s: Error! disabling ln8411: ret(%d)\n", __func__, ret);
			ln8411->charging_state = DC_STATE_ERROR;
			ln8411_vote_dc_avail(ln8411, 0, 1);
		}
		ln8411->timer_period = 0;
	}

	if (!ln8411->ftm_mode)
		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 	 msecs_to_jiffies(ln8411->timer_period));
	mutex_unlock(&ln8411->lock);
	return ret;
}

/* Enter direct charging algorithm */
static int ln8411_start_direct_charging(struct ln8411_charger *ln8411)
{
	struct ln8411_chg_stats *chg_data = &ln8411->chg_data;
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);
	mutex_lock(&ln8411->lock);

	/* configure DC charging type for the requested index */
	if (!ln8411->ftm_mode)
		ret = ln8411_set_ta_type(ln8411, ln8411->pps_index);
	dev_info(ln8411->dev, "%s: Current ta_type=%d, chg_mode=%d\n", __func__,
		ln8411->ta_type, ln8411->chg_mode);
	if (ret < 0)
		goto error_done;

	/* wake lock */
	__pm_stay_awake(ln8411->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = ln8411_preset_dcmode(ln8411);
	if (ret == 0) {
		/* Configure the TA  and start charging */
		ln8411->timer_id = TIMER_PDMSG_SEND;
		ln8411->timer_period = 0;

		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));
	}

error_done:
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);

	ln8411_chg_stats_update(chg_data, ln8411);
	mutex_unlock(&ln8411->lock);
	return ret;
}

/* Check Vbat minimum level to start direct charging */
static int ln8411_check_vbatmin(struct ln8411_charger *ln8411)
{
	int ret = 0, vbat;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);

	mutex_lock(&ln8411->lock);

	if (ln8411->charging_state != DC_STATE_CHECK_VBAT)
		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CHECK_VBAT);

	ln8411->charging_state = DC_STATE_CHECK_VBAT;

	if (ln8411->hw_init_done == false)
		if (!ln8411_hw_init(ln8411))
			ln8411->hw_init_done = true;

	vbat = ln8411_read_adc(ln8411, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
		goto error;
	} else if (vbat <= LN8411_DC_VBAT_MIN) {
		ret = -EAGAIN;
		ln8411->error = LN8411_ERROR_LOW_VBATT;
		goto error;
	}

	/* wait for hw init and CPM to send in the params */
	if (ln8411->cc_max < 0 || ln8411->fv_uv < 0 || !ln8411->hw_init_done) {
		dev_info(ln8411->dev, "%s: not yet fv_uv=%d, cc_max=%d vbat=%d, hw_init_done=%d\n",
			 __func__, ln8411->fv_uv, ln8411->cc_max, vbat, ln8411->hw_init_done);

		/* retry again after 1sec */
		ln8411->timer_id = TIMER_VBATMIN_CHECK;
		ln8411->timer_period = LN8411_VBATMIN_CHECK_T;
		ln8411->retry_cnt += 1;
	} else {
		logbuffer_prlog(ln8411, LOGLEVEL_INFO,
				"%s: starts at fv_uv=%d, cc_max=%d vbat=%d (min=%d)",
				__func__, ln8411->fv_uv, ln8411->cc_max, vbat,
				LN8411_DC_VBAT_MIN);

		ln8411->timer_id = TIMER_PRESET_DC;
		ln8411->timer_period = 0;
		ln8411->retry_cnt = 0; /* start charging */
	}

	/* timeout for VBATMIN or charging parameters */
	if (ln8411->retry_cnt > LN8411_MAX_RETRY_CNT) {
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: TIMEOUT fv_uv=%d, cc_max=%d vbat=%d limit=%d",
				__func__, ln8411->fv_uv, ln8411->cc_max, vbat,
				LN8411_DC_VBAT_MIN);
		ret = -ETIMEDOUT;
	} else {
		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));
	}


error:
	mutex_unlock(&ln8411->lock);
	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/*
 * return 1 if apdo switch, 0 if no switch. < 0 on err.
 * TODO : lower input current per TCPC spec
 */
static int ln8411_check_apdo_switch(struct ln8411_charger *ln8411)
{
	unsigned int ta_max_vol, ta_max_cur, ta_objpos;
	unsigned int new_ta_cur, new_ta_max_cur, val;
	int ret;

	dev_dbg(ln8411->dev, "%s: START: ta_vol: %d, prev_ta_vol: %d, ta_cur: %d, prev_ta_cur: %d\n",
		__func__, ln8411->ta_vol, ln8411->prev_ta_vol, ln8411->ta_cur, ln8411->prev_ta_cur);

	ta_max_vol = ln8411->ta_vol;
	ta_max_cur = ln8411->ta_cur;

	ret = ln8411_get_apdo_index(ln8411, &ta_max_vol, &ta_max_cur, &ta_objpos);
	if (ret) {
		dev_dbg(ln8411->dev, "%s: error getting apdo index (%d)\n", __func__, ret);
		return ret;
	}

	if (ln8411->prev_ta_cur == ln8411->ta_cur && ln8411->prev_ta_vol == ln8411->ta_vol) {
		ln8411->ta_objpos = ta_objpos;
		return 0;
	}

	if (ta_objpos == ln8411->ta_objpos) {
		dev_dbg(ln8411->dev, "%s: stay at apdo %d\n", __func__, ta_objpos);
		return 0;
	}

	if (ln8411->prev_ta_cur < ln8411->ta_cur) {
		/* Should never happen as we limit to ta_max_cur */
		dev_err(ln8411->dev, "%s: ta_cur: %d > ta_max_cur %d causing APDO switch\n",
				__func__, ln8411->ta_cur, ln8411->ta_max_cur);
		ln8411->ta_cur = ln8411->ta_max_cur;
		ret = -EINVAL;
	} else if (ln8411->prev_ta_vol != ln8411->ta_vol) {
		const long power = (ln8411->prev_ta_cur / 1000) * (ln8411->prev_ta_vol / 1000);

		new_ta_cur = (power / (ln8411->ta_vol / 1000)) * 1000;
		new_ta_max_cur = new_ta_cur;
		ta_max_vol = ln8411->ta_vol;
		dev_dbg(ln8411->dev, "%s: find new ta_cur: ta_vol: %d, ta_cur: %d\n",
			__func__, ln8411->ta_vol, new_ta_cur);
		ret = ln8411_get_apdo_index(ln8411, &ta_max_vol, &new_ta_max_cur, &ta_objpos);
		if (ret) {
			new_ta_max_cur = 0;
			ta_max_vol = ln8411->ta_vol;
			ret = ln8411_get_apdo_index(ln8411, &ta_max_vol, &new_ta_max_cur, &ta_objpos);
			if (ret) {
				dev_err(ln8411->dev, "No available APDO to switch to (%d)\n", ret);
			} else {
				/* APDO can't provide needed ta_cur, so limit to max */
				val = new_ta_max_cur / PD_MSG_TA_CUR_STEP;
				ln8411->ta_cur = val * PD_MSG_TA_CUR_STEP;
				ln8411->ta_max_cur = ln8411->ta_cur;
			}
		} else {
			val = new_ta_cur / PD_MSG_TA_CUR_STEP;
			ln8411->ta_cur = val * PD_MSG_TA_CUR_STEP;
			ln8411->ta_max_cur = new_ta_max_cur;
		}
	}

	if (!ret && ta_objpos != ln8411->ta_objpos) {
		const int temp_cur = ln8411->ta_cur;

		dev_info(ln8411->dev, "ta_vol: %d->%d, ta_cur: %d->%d, ta_pos: %d->%d\n",
			ln8411->prev_ta_vol, ln8411->ta_vol, ln8411->prev_ta_cur, ln8411->ta_cur,
			ln8411->ta_objpos, ta_objpos);

		ln8411->ta_objpos = ta_objpos;

		/* Send one message immediately */
		/* force only voltage change */
		ln8411->ta_cur = ln8411->prev_ta_cur;
		ret = ln8411_send_pd_message(ln8411, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			return ret;
		ln8411->ta_cur = temp_cur;
		ret = 1;
	}

	return ret;
}

static int ln8411_send_message(struct ln8411_charger *ln8411)
{
	int val, ret;
	const int timer_id = ln8411->timer_id;

	/* Go to the next state */
	mutex_lock(&ln8411->lock);

	dev_dbg(ln8411->dev, "%s: ====== START ======= \n", __func__);

	if (ln8411->ftm_mode)
		goto skip_pps;

	/* Adjust TA current and voltage step */
	if (ln8411->ta_type == TA_TYPE_WIRELESS) {
		/* RX voltage resolution is 40mV */
		val = ln8411->ta_vol / WCRX_VOL_STEP;
		ln8411->ta_vol = val * WCRX_VOL_STEP;

		/* Set RX voltage */
		dev_dbg(ln8411->dev, "%s: ta_type=%d, ta_vol=%d\n", __func__,
			 ln8411->ta_type, ln8411->ta_vol);
		ret = ln8411_send_rx_voltage(ln8411, WCRX_REQUEST_VOLTAGE);
	} else {
		/* PPS voltage resolution is 20mV */
		val = ln8411->ta_vol / PD_MSG_TA_VOL_STEP;
		ln8411->ta_vol = val * PD_MSG_TA_VOL_STEP;
		/* PPS current resolution is 50mA */
		val = ln8411->ta_cur / PD_MSG_TA_CUR_STEP;
		ln8411->ta_cur = val * PD_MSG_TA_CUR_STEP;
		/* PPS minimum current is 1000mA */
		if (ln8411->ta_cur < LN8411_TA_MIN_CUR)
			ln8411->ta_cur = LN8411_TA_MIN_CUR;

		dev_dbg(ln8411->dev, "%s: ta_type=%d, ta_vol=%d ta_cur=%d\n", __func__,
			 ln8411->ta_type, ln8411->ta_vol, ln8411->ta_cur);

		if (!ln8411->prev_ta_cur)
			ln8411->prev_ta_cur = ln8411->ta_cur;
		if (!ln8411->prev_ta_vol)
			ln8411->prev_ta_vol = ln8411->ta_vol;

		ln8411_check_apdo_switch(ln8411);
		/* Send PD Message */
		ret = ln8411_send_pd_message(ln8411, PD_MSG_REQUEST_APDO);
		if (ret >= 0) {
			ln8411->prev_ta_cur = ln8411->ta_cur;
			ln8411->prev_ta_vol = ln8411->ta_vol;
		}

	}

skip_pps:
	switch (ln8411->charging_state) {
	case DC_STATE_PRESET_DC:
		ln8411->timer_id = TIMER_PRESET_CONFIG;
		break;
	case DC_STATE_ADJUST_CC:
		ln8411->timer_id = TIMER_ADJUST_CCMODE;
		break;
	case DC_STATE_CC_MODE:
		ln8411->timer_id = TIMER_CHECK_CCMODE;
		break;
	case DC_STATE_START_CV:
		ln8411->timer_id = TIMER_ENTER_CVMODE;
		break;
	case DC_STATE_CV_MODE:
		ln8411->timer_id = TIMER_CHECK_CVMODE;
		break;
	case DC_STATE_ADJUST_TAVOL:
		ln8411->timer_id = TIMER_ADJUST_TAVOL;
		break;
	case DC_STATE_ADJUST_TACUR:
		ln8411->timer_id = TIMER_ADJUST_TACUR;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		dev_err(ln8411->dev, "%s: Error-send_pd_message to %d (%d)\n",
			__func__, ln8411->ta_type, ret);
		ln8411->timer_id = TIMER_CHECK_ACTIVE;
	}

	/* Ensure both TA voltage and current get set before enabling charging */
	if (ln8411->ftm_mode)
		ln8411->timer_period = 0;
	else if (ln8411->timer_id == TIMER_PRESET_CONFIG)
		ln8411->timer_period = LN8411_TA_CONFIG_WAIT_T;
	else if (ln8411->ta_type == TA_TYPE_WIRELESS)
		ln8411->timer_period = LN8411_PDMSG_WLC_WAIT_T;
	else
		ln8411->timer_period = LN8411_PDMSG_WAIT_T;

	logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
			"%s: charging_state=%u timer_id:%d->%d ret=%d",
			__func__, ln8411->charging_state,
			timer_id, ln8411->timer_id, ret);

	mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
			 msecs_to_jiffies(ln8411->timer_period));

	dev_dbg(ln8411->dev, "%s: End: timer_id=%d timer_period=%lu\n", __func__,
		 ln8411->timer_id, ln8411->timer_period);

	mutex_unlock(&ln8411->lock);
	return ret;
}

/* delayed work function for charging timer */
static void ln8411_timer_work(struct work_struct *work)
{
	struct ln8411_charger *ln8411 =
		container_of(work, struct ln8411_charger, timer_work.work);
	unsigned int charging_state;
	int timer_id;
	int ret = 0;
	bool retries = false;

	dev_dbg(ln8411->dev, "%s: ========= START =========\n", __func__);

	/* TODO: remove locks from the calls and run all of this locked */
	mutex_lock(&ln8411->lock);

	ln8411_chg_stats_update(&ln8411->chg_data, ln8411);
	charging_state = ln8411->charging_state;
	timer_id = ln8411->timer_id;

	dev_dbg(ln8411->dev, "%s: timer id=%d, charging_state=%u\n", __func__,
		 ln8411->timer_id, charging_state);

	mutex_unlock(&ln8411->lock);

	switch (timer_id) {

	/* charging_state <- DC_STATE_CHECK_VBAT */
	case TIMER_VBATMIN_CHECK:
		ret = ln8411_check_vbatmin(ln8411);
		if (ret < 0)
			goto error;
		break;

	/* charging_state <- DC_STATE_PRESET_DC */
	case TIMER_PRESET_DC:
		ret = ln8411_start_direct_charging(ln8411);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	preset configuration, start charging
	 */
	case TIMER_PRESET_CONFIG:
		ret = ln8411_preset_config(ln8411);
		if (ret < 0)
			goto error;
		break;

	/*
	 * charging_state <- DC_STATE_PRESET_DC
	 *	150 ms after preset_config
	 */
	case TIMER_CHECK_ACTIVE:
		ret = ln8411_check_active_state(ln8411);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ADJUST_CCMODE:
		ret = ln8411_charge_adjust_ccmode(ln8411);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CCMODE:
		ret = ln8411_charge_ccmode(ln8411);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = ln8411_charge_start_cvmode(ln8411);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CHECK_CVMODE:
		ret = ln8411_charge_cvmode(ln8411);
		if (ret < 0)
			goto error;
		break;

	case TIMER_PDMSG_SEND:
		ret = ln8411_send_message(ln8411);
		if (ret < 0)
			goto error;
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TAVOL:
		mutex_lock(&ln8411->lock);

		if (ln8411->ta_type == TA_TYPE_WIRELESS)
			ret = ln8411_adjust_rx_voltage(ln8411);
		else
			ret = ln8411_adjust_ta_voltage(ln8411);
		if (ret < 0) {
			mutex_unlock(&ln8411->lock);
			goto error;
		}

		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));
		mutex_unlock(&ln8411->lock);
		break;

	/* called from 2 contexts */
	case TIMER_ADJUST_TACUR:
		mutex_lock(&ln8411->lock);
		ret = ln8411_adjust_ta_current(ln8411);
		if (ret < 0) {
			mutex_unlock(&ln8411->lock);
			goto error;
		}

		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));
		mutex_unlock(&ln8411->lock);
		break;

	case TIMER_ID_NONE:
		ret = ln8411_stop_charging(ln8411);
		if (ret < 0)
			goto error;
		break;

	default:
		break;
	}

	/* Check the charging state again */
	if (ln8411->charging_state == DC_STATE_NO_CHARGING) {
		cancel_delayed_work(&ln8411->timer_work);
		cancel_delayed_work(&ln8411->pps_work);
	}

	dev_dbg(ln8411->dev, "%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld\n",
		 __func__, timer_id, ln8411->timer_id, charging_state,
		 ln8411->charging_state, ln8411->timer_period);

	return;

error:
	dev_dbg(ln8411->dev, "%s: ========= ERROR =========\n", __func__);
	logbuffer_prlog(ln8411, LOGLEVEL_ERR,
			"%s: timer_id=%d->%d, charging_state=%u->%u, period=%ld err=%d ret=%d "
			"ucp_count:%d ucp_debounce:%d low_batt_count:%d",
			__func__, timer_id, ln8411->timer_id, charging_state,
			ln8411->charging_state, ln8411->timer_period, ln8411->error, ret,
			ln8411->ibus_ucp_retry_cnt, ln8411->ibus_ucp_debounce_cnt,
			ln8411->low_batt_retry_cnt);

	if (ret == -EAGAIN && ln8411_err_is_ucp(ln8411, &retries) && retries) {
		/* Retry for IBUS UCP case */
		ln8411->ibus_ucp_retry_cnt--;
		ln8411->ibus_ucp_debounce_cnt = LN8411_MAX_IBUS_UCP_DEBOUNCE_COUNT;
		ret = ln8411_set_status_disable_charging(ln8411);
		if (ret) {
			dev_err(ln8411->dev, "%s: unable to disable charging for retry (%d)\n",
				__func__, ret);
		} else {
			ret = ln8411_set_status_charging(ln8411);
			if (ret)
				dev_err(ln8411->dev, "%s: unable to enable charging for retry (%d)\n",
				__func__, ret);
		}

		if (!ret)
			mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
					msecs_to_jiffies(LN8411_ENABLE_WLC_DELAY_T));
	} else if (ret == -EAGAIN && ln8411_err_is_low_batt(ln8411, &retries) && retries) {
		ln8411->low_batt_retry_cnt--;
		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(LN8411_ENABLE_WLC_DELAY_T));
	} else {
		ln8411_stop_charging(ln8411);
		if (ret == -EAGAIN && ((ln8411_err_is_ucp(ln8411, &retries) && !retries) ||
                                       (ln8411_err_is_low_batt(ln8411, &retries) && !retries))) {
			dev_err(ln8411->dev, "%s: retry failed err:%d\n", __func__, ln8411->error);

			ln8411->charging_state = DC_STATE_ERROR;
			ln8411_vote_dc_avail(ln8411, 0, 1);
		}
	}
}

/* delayed work function for resetting DC chip */
static void ln8411_init_hw_work(struct work_struct *work)
{
	struct ln8411_charger *ln8411 = container_of(work,
					struct ln8411_charger, init_hw_work.work);
	int ret;

	mutex_lock(&ln8411->lock);
	/* Integration guide V1.0 Section 4 */
	ret = ln8411_hw_init(ln8411);
	if (ret) {
		dev_err(ln8411->dev, "Error initializing hw %d\n", ret);
		goto error;
	}

	ln8411->hw_init_done = true;
error:
	mutex_unlock(&ln8411->lock);
	return;
}

/* delayed work function for pps periodic timer */
static void ln8411_pps_request_work(struct work_struct *work)
{
	struct ln8411_charger *ln8411 = container_of(work,
					struct ln8411_charger, pps_work.work);
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);
	dev_dbg(ln8411->dev, "%s: = charging_state=%u == \n", __func__,
		 ln8411->charging_state);

	if (!ln8411->ftm_mode)
		ret = ln8411_send_pd_message(ln8411, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		dev_err(ln8411->dev, "%s: Error-send_pd_message\n", __func__);

	/* TODO: do other background stuff */

	dev_dbg(ln8411->dev, "%s: ret=%d\n", __func__, ret);
}

static int ln8411_soft_reset(struct ln8411_charger *ln8411)
{
	int ret;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_RESET);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_TEST_MODE_CTRL, LN8411_SOFT_RESET_REQ);
	if (ret)
		return ret;

	msleep(50);
	if (ln8411->chip_info.chip_rev == 1)
		msleep(100);

	/* No need to relock after reset */
	return ret;
}

/* Caller must poll ADC_DONE_FLAG till ADC is enabled */
static int ln8411_cfg_adc(struct ln8411_charger *ln8411)
{
	int ret;

	/* Disable unused ADC channels */
	ret = regmap_write(ln8411->regmap, LN8411_ADC_FN_DISABLE1, LN8411_VUSB_ADC_DIS
			   | LN8411_VWPC_ADC_DIS | LN8411_IBAT_ADC_DIS | LN8411_TSBAT_ADC_DIS);
	if (ret)
		return ret;

	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CTRL2, 0xC0);

#ifdef POLL_ADC
	ret = regmap_update_bits(ln8411->regmap, LN8411_ADC_CTRL, 0xc9, LN8411_ADC_EN);
#else
	ret = regmap_update_bits(ln8411->regmap, LN8411_ADC_CTRL, 0xc9,
				 LN8411_ADC_EN | LN8411_ADC_DONE_MASK);
#endif
	if (ret)
		return ret;

#ifndef POLL_ADC
	msleep(350);
#endif

	return ret;
}

/* Integration guide V1.0 Section 4 */
static int ln8411_hw_init(struct ln8411_charger *ln8411)
{
	int ret;
#ifdef POLL_ADC
	int retry = ADC_EN_RETRIES;
	int adc_done;
#endif

	/* Reset the chip - Integration guide V1.0 Section 4.1 */
	dev_info(ln8411->dev, "%s: reset chip\n", __func__);
	ret = ln8411_soft_reset(ln8411);
	if (ret)
		goto error_done;

	if (ln8411->pdata->irq_gpio >= 0) {
		ret = ln8411_irq_init(ln8411);
		if (ret < 0)
			dev_warn(ln8411->dev, "%s: failed to initialize IRQ: %d\n", __func__, ret);
		else
			disable_irq(ln8411->client->irq);
	}

	/* Integration guide V1.2 Section 4.2 */
	dev_dbg(ln8411->dev, "%s: Enable TSBAT_EN_PIN\n", __func__);
	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL4, LN8411_TSBAT_EN_PIN);
	if (ret)
		goto error_done;

	/* Turn OFF both OVP and WPC gates to prevent wrong input detection */
	dev_dbg(ln8411->dev, "%s: turn OFF gates\n", __func__);
	ret = regmap_clear_bits(ln8411->regmap, LN8411_CTRL1, LN8411_OVPGATE_EN | LN8411_WPCGATE_EN);
	if (ret)
		goto error_done;

	/* 685kHz, enable spread spectrum */
	ret = regmap_update_bits(ln8411->regmap, LN8411_CTRL2, 0xfe, 0x9A);
	if (ret)
		goto error_done;

	/* unlock private register space */
	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
	if (ret)
		goto error_done;

	ret = regmap_write(ln8411->regmap, LN8411_SC_DITHER_CTRL, 0x7F);
	if (ret)
		goto error_done;

	dev_dbg(ln8411->dev, "%s: clear latched sts\n", __func__);
	ret = regmap_set_bits(ln8411->regmap, LN8411_ADC_CFG_2, LN8411_CLEAR_LATCHED_STS);
	if (ret)
		goto error_done;

	dev_dbg(ln8411->dev, "%s: disable alarms\n", __func__);
	ret = regmap_clear_bits(ln8411->regmap, LN8411_ALARM_CTRL, 0x1c);
	if (ret)
		goto error_done;

	if (ln8411->pdata->si_fet_ovp_drive) {
		dev_dbg(ln8411->dev, "%s: set safety switch to 10V\n", __func__);
		/* Set Safety switch drive for 10V Si FET */
		ret = regmap_set_bits(ln8411->regmap, LN8411_OVPGATE_CTRL_0, LN8411_OVPFETDR_V_CFG);
		if (ret)
			goto error_done;
	}

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	if (ret)
		goto error_done;

	/* Set protection thresholds */
	ln8411->iin_reg = 0;
	ln8411->vfloat_reg = 0;

	dev_dbg(ln8411->dev, "%s: set_vbat_ovp\n", __func__);
	ret = ln8411_set_vbat_ovp(ln8411, VBAT_OVP_DFT);
	if (ret)
		goto error_done;

	dev_dbg(ln8411->dev, "%s: set ibus ucp\n", __func__);
	ret = regmap_write(ln8411->regmap, LN8411_IBUS_UCP, 0x10);
	if (ret)
		goto error_done;


	if (ln8411->chip_info.chip_rev == 1) {
		dev_dbg(ln8411->dev, "%s: pmid2out ovp to 13 for A1%%\n", __func__);
		ret = regmap_update_bits(ln8411->regmap, LN8411_PMID2OUT_OVP, 0x9f, 0x4);
	} else {
		dev_dbg(ln8411->dev, "%s: pmid2out ovp to 16%% for B0 and above\n", __func__);
		ret = regmap_update_bits(ln8411->regmap, LN8411_PMID2OUT_OVP, 0x9f, 0x5);
	}
	if (ret)
		goto error_done;

	dev_dbg(ln8411->dev, "%s: clear int flags\n", __func__);
	ret = regmap_set_bits(ln8411->regmap, LN8411_INT_MASK_2, LN8411_PAUSE_INT_UPDATE);
	if (ret)
		goto error_done;

	ret = regmap_set_bits(ln8411->regmap, LN8411_INT_MASK_2, LN8411_CLEAR_INT);
	if (ret)
		goto error_done;

	msleep(5);

	ret = regmap_clear_bits(ln8411->regmap,LN8411_INT_MASK_2, LN8411_CLEAR_INT);
	if (ret)
		goto error_done;

	ret = regmap_clear_bits(ln8411->regmap, LN8411_INT_MASK_2, LN8411_PAUSE_INT_UPDATE);
	if (ret)
		goto error_done;

	/* Enable ADC */
	dev_dbg(ln8411->dev, "%s: Enable ADC\n", __func__);
	ret = ln8411_cfg_adc(ln8411);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error configuring adc\n", __func__);
		goto error_done;
	}

#ifdef POLL_ADC
	retry = ADC_EN_RETRIES;
	do {
		msleep(10);
		retry--;
		regmap_read(ln8411->regmap, LN8411_COMP_FLAG1, &adc_done);
		adc_done &= LN8411_ADC_DONE_FLAG;
	} while (retry && adc_done == 0);

	if (!adc_done)
		dev_err(ln8411->dev, "%s: Error enabling adc\n", __func__);
#endif
	if (!ret)
		dev_dbg(ln8411->dev, "HW init done");
error_done:
	return ret;
}

static irqreturn_t ln8411_interrupt_handler(int irq, void *data)
{
	struct ln8411_charger *ln8411 = data;
	bool handled = true;
	int ret;
	unsigned int val;

	ret = regmap_read(ln8411->regmap, LN8411_INT_FLAG, &val);
	if (ret) {
		dev_err(ln8411->dev, "%s: Error reading LN8411_INT_FLAG: %d\n",
			__func__, ret);
		goto exit_done;
	}
	dev_dbg(ln8411->dev, "%s: FLG %d\n", __func__, val);

	ret = regmap_read(ln8411->regmap, LN8411_FAULT3_STS, &val);
	if (val & LN8411_VBAT_ALARM_STS) {
		dev_info(ln8411->dev, "%s: In VFLT LOOP\n", __func__);
	}

	if (val & LN8411_IBUS_ALARM_STS) {
		dev_info(ln8411->dev, "%s: In IIN LOOP\n", __func__);
	}

	ret = regmap_read(ln8411->regmap, LN8411_INT_MASK, &val);
	if (ret < 0) {
		dev_err(ln8411->dev, "%s: Error reading interrupts enable: %d\n",
			__func__, ret);
		goto exit_done;
	}
	dev_dbg(ln8411->dev, "%s: Interrupt Mask: %d\n", __func__, val);

	/* Clear the interrupts */
	/* TODO */

exit_done:
	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int ln8411_irq_init(struct ln8411_charger *ln8411)
{
	const struct ln8411_platform_data *pdata = ln8411->pdata;
	int ret, irq;

	irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, ln8411->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, ln8411_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   ln8411->client->name, ln8411);
	if (ret < 0)
		goto fail_gpio;


	/* Mask all IRQ for the time being */
	ret = regmap_clear_bits(ln8411->regmap, LN8411_INT_MASK, LN8411_VOUT_INSERT_MASK |
			   LN8411_VWPC_INSERT_MASK | LN8411_VUSB_INSERT_MASK);
	if (ret)
		goto fail_write;

	ln8411->client->irq = irq;
	return 0;

fail_write:
	free_irq(irq, ln8411);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	ln8411->client->irq = 0;
	return ret;
}

/* Returns the input current limit programmed into the charger in uA. */
int ln8411_input_current_limit(struct ln8411_charger *ln8411)
{
	if (!ln8411->mains_online)
		return -ENODATA;

	return ln8411->iin_reg;
}

/* Returns the constant charge current requested from GCPM */
static int get_const_charge_current(struct ln8411_charger *ln8411)
{
	/* Charging current cannot be controlled directly */
	return ln8411->cc_max;
}

/* Return the constant charge voltage programmed into the charger in uV. */
static int ln8411_const_charge_voltage(struct ln8411_charger *ln8411)
{
	if (!ln8411->mains_online)
		return -ENODATA;

	return ln8411->vfloat_reg;
}

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

/* index is the PPS source to use */
static int ln8411_set_charging_enabled(struct ln8411_charger *ln8411, int index)
{
	int ret = 0;
	if (index < 0 || index >= PPS_INDEX_MAX)
		return -EINVAL;

	mutex_lock(&ln8411->lock);

	/* Done is detected in CV when iin goes UNDER topoff. */
	if (ln8411->charging_state == DC_STATE_CHARGING_DONE)
		index = 0;

	if (index == 0) {

		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: stop pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, ln8411->pps_index, index,
				ln8411->charging_state,
				ln8411->timer_id);

		/* this is the same as stop charging */
		ln8411->pps_index = 0;

		cancel_delayed_work(&ln8411->timer_work);
		cancel_delayed_work(&ln8411->pps_work);

		/* will call ln8411_stop_charging() in timer_work() */
		ln8411->timer_id = TIMER_ID_NONE;
		ln8411->timer_period = 0;
		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));
	} else if (ln8411->charging_state == DC_STATE_NO_CHARGING) {
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: start pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, ln8411->pps_index, index,
				ln8411->charging_state,
				ln8411->timer_id);

		/* Start Direct Charging on Index */
		ln8411->dc_start_time = get_boot_sec();
		ln8411_chg_stats_init(&ln8411->chg_data);
		ln8411->pps_index = index;

		dev_info(ln8411->dev, "%s: charging_state=%u->%u\n", __func__,
			 ln8411->charging_state, DC_STATE_CHECK_VBAT);

		/* PD is already in PE_SNK_STATE */
		ln8411->charging_state = DC_STATE_CHECK_VBAT;
		ln8411->timer_id = TIMER_VBATMIN_CHECK;
		ln8411->timer_period = 0;
		mod_delayed_work(ln8411->dc_wq, &ln8411->timer_work,
				 msecs_to_jiffies(ln8411->timer_period));

		/* Set the initial charging step */
		power_supply_changed(ln8411->mains);
	} else if (ln8411->charging_state == DC_STATE_ERROR) {
		logbuffer_prlog(ln8411, LOGLEVEL_DEBUG,
				"%s: error pps_idx=%d->%d charging_state=%d timer_id=%d",
				__func__, ln8411->pps_index, index,
		ln8411->charging_state,
		ln8411->timer_id);
		ret = -EINVAL;
	}

	mutex_unlock(&ln8411->lock);

	return ret;
}

/* call holding mutex_lock(&ln8411->lock); */
static int ln8411_init_1_2_mode(struct ln8411_charger *ln8411)
{
	int ret, ret1;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);

	if (ln8411->chg_mode != CHG_NO_DC_MODE) {
		dev_info(ln8411->dev, "%s: chg_mode is not NO_DC_MODE. Not initing 1_2 mode=%d\n",
			 __func__, ln8411->chg_mode);
		ret = -ENOTSUPP;
		goto error;
	}
	ln8411->chg_mode = CHG_1TO2_DC_MODE;

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error setting EN_SW_OVERRIDE (%d)\n", __func__, ret);
		goto error;
	}

	ret = ln8411_set_mode(ln8411, CHG_1TO2_DC_MODE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error setting Rev 1:2 mode (%d)\n", __func__, ret);
		goto error;
	}

	ret = regmap_clear_bits(ln8411->regmap, LN8411_CFG_1, LN8411_DEVICE_MODE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error clearing DEVICE_MODE (%d)\n", __func__, ret);
		goto error;
	}

	ret =regmap_set_bits(ln8411->regmap, LN8411_REG_49, LN8411_REVERT_LSNS);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error setting LSNS (%d)\n", __func__, ret);
		goto error;
	}

error:
	ret1 = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	if (ret1) {
		dev_info(ln8411->dev, "%s: Error locking private regs (%d)\n", __func__, ret1);
		ret = ret1;
	}

	return ret;
}

/* call holding mutex_lock(&ln8411->lock); */
static int ln8411_enable_1_2_mode(struct ln8411_charger *ln8411)
{
	int ret, ret1;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);

	ret = ln8411_set_prot_by_chg_mode(ln8411);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error settings protections (%d)\n", __func__, ret);
		goto error;
	}

	ret = ln8411_set_status_charging(ln8411);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error starting charging (%d)\n", __func__, ret);
		goto error;
	}

	msleep(200);

	ret = ln8411_check_error(ln8411);
	if (ret)
		goto error;

	/* Enable PMID2OUT_UVP */
	ret = regmap_update_bits(ln8411->regmap, LN8411_PMID2OUT_UVP, 0x9f, PMID2OUT_UVP_DFT_EN_1_2);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error enabling PMID2OUT_UVP (%d)\n", __func__, ret);
		goto error;
	}

	ret = regmap_set_bits(ln8411->regmap, LN8411_CTRL1, LN8411_WPCGATE_EN);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error enabling WPCGATE (%d)\n", __func__, ret);
		goto error;
	}

	msleep(20);

	ret = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_EN_SW_OVERRIDE);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error setting EN_SW_OVERRIDE (%d)\n", __func__, ret);
		goto error;
	}

	/* Enable WPC_UVP */
	ret = regmap_write(ln8411->regmap, LN8411_LION_COMP_CTRL_2, 0x0);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error enabling WPC_UVP (%d)\n", __func__, ret);
		goto error;
	}

	ret = ln8411_check_error(ln8411);
	if (ret)
		goto error;

error:
	ret1 = ln8411_set_lion_ctrl(ln8411, LN8411_LION_CTRL_LOCK);
	if (ret1) {
		dev_info(ln8411->dev, "%s: Error locking private regs (%d)\n", __func__, ret1);
		ret = ret1;
	}

	return ret;
}

/* call holding mutex_lock(&ln8411->lock); */
static int ln8411_stop_1_2_mode(struct ln8411_charger *ln8411)
{
	int ret, ret1;

	dev_dbg(ln8411->dev, "%s: =========START========= mode: %d\n", __func__, ln8411->chg_mode);
        if (ln8411->chg_mode != CHG_1TO2_DC_MODE)
               return 0;

	ret = ln8411_set_status_disable_charging(ln8411);
	if (ret < 0)
		goto error;

	ln8411->chg_mode = CHG_NO_DC_MODE;

error:
	ln8411->hw_init_done = false;
	ret1 = ln8411_hw_init(ln8411);
	if (ret1 < 0) {
		dev_info(ln8411->dev, "%s: Error initializing HW (%d)\n", __func__, ret1);
		ret = ret1;
	} else {
		ln8411->hw_init_done = true;
	}

	return ret;
}

static int ln8411_start_1_2_mode(struct ln8411_charger *ln8411)
{
	int ret;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);
	mutex_lock(&ln8411->lock);

	ret = ln8411_init_1_2_mode(ln8411);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error initializing 1_2 mode (%d)\n", __func__, ret);
		if (ret == -ENOTSUPP) {
			mutex_unlock(&ln8411->lock);
			return ret;
		}

		goto error;
	}

	ret = ln8411_enable_1_2_mode(ln8411);
	if (ret) {
		dev_info(ln8411->dev, "%s: Error enabling 1_2 mode (%d)\n", __func__, ret);
		goto error;
	}

error:
	if (ret) {
		int rc = ln8411_stop_1_2_mode(ln8411);

		if (rc)
			dev_info(ln8411->dev, "%s: Error disabling 1_2 mode (%d)\n", __func__, rc);
	}

	mutex_unlock(&ln8411->lock);
	return ret;
}

#if IS_ENABLED(CONFIG_GPIOLIB)
static int ln8411_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int ln8411_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ln8411_charger *ln8411 = gpiochip_get_data(chip);
	int ret = 0;

	switch (offset) {
	/*
	 * Get LN8411_GPIO_1_2_EN returns 1 when it's not in forward mode, allowing
	 * reverse mode to be enabled. WLC will wait until this gpio is 1 before
	 * attempting to enable reverse mode.
	 */
	case LN8411_GPIO_1_2_EN:
		mutex_lock(&ln8411->lock);
		ret = (ln8411->chg_mode == CHG_NO_DC_MODE) || (ln8411->chg_mode == CHG_1TO2_DC_MODE);

		if (ln8411->chg_mode == CHG_1TO2_DC_MODE)
			ln8411_check_error(ln8411);
		mutex_unlock(&ln8411->lock);
		break;
	default:
		return -EINVAL;
		break;
	}

	pr_debug("%s: GPIO offset=%d ret:%d\n", __func__, offset, ret);
	return ret;
}

static void ln8411_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ln8411_charger *ln8411 = gpiochip_get_data(chip);
	int ret = 0;

	switch (offset) {
	case LN8411_GPIO_1_2_EN:
		if (value) {
			ret = ln8411_start_1_2_mode(ln8411);
		} else {
			mutex_lock(&ln8411->lock);
			ret = ln8411_stop_1_2_mode(ln8411);
			mutex_unlock(&ln8411->lock);
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pr_debug("%s: GPIO offset=%d value=%d ret:%d\n", __func__,
		 offset, value, ret);

	if (ret < 0)
		dev_err(&ln8411->client->dev, "GPIO%d: value=%d ret:%d\n", offset, value, ret);
}

static void ln8411_gpio_init(struct ln8411_charger *ln8411)
{
	ln8411->gpio.owner = THIS_MODULE;
	ln8411->gpio.label = "ln8411_gpio";
	ln8411->gpio.get_direction = ln8411_gpio_get_direction;
	ln8411->gpio.get = ln8411_gpio_get;
	ln8411->gpio.set = ln8411_gpio_set;
	ln8411->gpio.base = -1;
	ln8411->gpio.ngpio = LN8411_NUM_GPIOS;
	ln8411->gpio.can_sleep = true;
}
#endif

static int ln8411_mains_set_property(struct power_supply *psy,
				      enum power_supply_property prop,
				      const union power_supply_propval *val)
{
	struct ln8411_charger *ln8411 = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);
	dev_dbg(ln8411->dev, "%s: prop=%d, val=%d\n", __func__, prop, val->intval);
	if (!ln8411->init_done)
		return -EAGAIN;

	switch (prop) {

	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval == 0) {
			ret = ln8411_stop_charging(ln8411);
			if (ret < 0)
				dev_err(ln8411->dev, "%s: cannot stop charging (%d)\n",
				       __func__, ret);

			ln8411->mains_online = false;
		} else if (ln8411->mains_online == false) {
			ln8411->mains_online = true;
		}

		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = ln8411_set_new_vfloat(ln8411, val->intval);
		break;

	/*
	 * dc charger cannot control charging current directly so need to control
	 * current on TA side resolving cc_max for TA_VOL*TA_CUT on vbat.
	 * NOTE: iin should be equivalent to iin = cc_max /2
	 */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = ln8411_set_new_cc_max(ln8411, val->intval);
		break;

	/* CURRENT MAX, same as IIN is really only set by the algo */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		dev_dbg(ln8411->dev, "%s: set iin %d, ignore\n", __func__, val->intval);
		break;

	/* allow direct setting, not used */
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&ln8411->lock);
		ret = ln8411_set_new_iin(ln8411, val->intval);
		mutex_unlock(&ln8411->lock);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int ln8411_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct ln8411_charger *ln8411 = power_supply_get_drvdata(psy);
	int intval, rc, ret = 0;

	if (!ln8411->init_done)
		return -EAGAIN;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ln8411->mains_online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ln8411_is_present(ln8411);
		if (val->intval < 0)
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = ln8411_const_charge_voltage(ln8411);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_const_charge_current(ln8411);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = ln8411_input_current_limit(ln8411);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* return the output current - uA unit */
		mutex_lock(&ln8411->lock);
		rc = ln8411_get_iin(ln8411, &val->intval);
		if (rc < 0)
			dev_err(ln8411->dev, "Invalid IIN ADC (%d)\n", rc);
		mutex_unlock(&ln8411->lock);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		mutex_lock(&ln8411->lock);
		intval = ln8411_read_adc(ln8411, ADCCH_VOUT);
		mutex_unlock(&ln8411->lock);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&ln8411->lock);
		intval = ln8411_read_adc(ln8411, ADCCH_VBAT);
		mutex_unlock(&ln8411->lock);
		if (intval < 0)
			return intval;
		val->intval = intval;
		break;

	/* TODO: read NTC temperature? */
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&ln8411->lock);
		val->intval = ln8411_read_adc(ln8411, ADCCH_DIETEMP);
		mutex_unlock(&ln8411->lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ln8411_get_charge_type(ln8411);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ln8411_get_status(ln8411);
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = ln8411_input_current_limit(ln8411);
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * GBMS not visible
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
 */
static enum power_supply_property ln8411_mains_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	/* same as POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT */
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int ln8411_mains_is_writeable(struct power_supply *psy,
				      enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int ln8411_gbms_mains_set_property(struct power_supply *psy,
					  enum gbms_property prop,
					  const union gbms_propval *val)
{
	struct ln8411_charger *ln8411 = power_supply_get_drvdata(psy);
	int ret = 0;

	dev_dbg(ln8411->dev, "%s: =========START=========\n", __func__);
	dev_dbg(ln8411->dev, "%s: prop=%d, val=%d\n", __func__, prop, val->prop.intval);
	if (!ln8411->init_done)
		return -EAGAIN;

	switch (prop) {

	case GBMS_PROP_CHARGING_ENABLED:
		ret = ln8411_set_charging_enabled(ln8411, val->prop.intval);
		break;

	case GBMS_PROP_CHARGE_DISABLE:
		dev_dbg(ln8411->dev, "%s: ChargeDisable %d, chg_state:%d\n", __func__,
			val->prop.intval, ln8411->charging_state);

		/* Reset state on disconnect event */
		if (val->prop.intval) {
			if (ln8411->charging_state == DC_STATE_ERROR)
				ln8411->charging_state = DC_STATE_NO_CHARGING;
			ln8411_vote_dc_avail(ln8411, 1, 1);

			ln8411->ibus_ucp_retry_cnt = LN8411_MAX_IBUS_UCP_RETRY_CNT;
			ln8411->ibus_ucp_debounce_cnt = LN8411_MAX_IBUS_UCP_DEBOUNCE_COUNT;
			ln8411->error = LN8411_ERROR_NONE;
			ln8411->low_batt_retry_cnt = LN8411_MAX_LOW_BATT_RETRY_CNT;
		}
		break;

	default:
		pr_debug("%s: route to ln8411_mains_set_property, psp:%d\n", __func__, prop);
		return -ENODATA;
	}

	dev_dbg(ln8411->dev, "%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int ln8411_gbms_mains_get_property(struct power_supply *psy,
					  enum gbms_property prop,
					  union gbms_propval *val)
{
	struct ln8411_charger *ln8411 = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int ret = 0;

	if (!ln8411->init_done)
		return -EAGAIN;

	switch (prop) {
	case GBMS_PROP_CHARGE_DISABLE:
		ret = ln8411_get_charging_enabled(ln8411);
		if (ret < 0)
			return ret;
		val->prop.intval = !ret;
		break;

	case GBMS_PROP_CHARGING_ENABLED:
		ret = ln8411_get_charging_enabled(ln8411);
		if (ret < 0)
			return ret;
		val->prop.intval = ret;
		break;

	case GBMS_PROP_CHARGE_CHARGER_STATE:
		ret = ln8411_get_chg_chgr_state(ln8411, &chg_state);
		if (ret < 0)
			return ret;
		val->int64val = chg_state.v;
		break;

	default:
		pr_debug("%s: route to ln8411_mains_get_property, psp:%d\n", __func__, prop);
		return -ENODATA;
	}

	return 0;
}

static int ln8411_gbms_mains_is_writeable(struct power_supply *psy,
					  enum gbms_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static const struct gbms_desc ln8411_mains_desc = {
	.psy_dsc.name		= "dc-mains",
	/* b/179246019 will not look online to Android */
	.psy_dsc.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.psy_dsc.properties	= ln8411_mains_properties,
	.psy_dsc.get_property	= ln8411_mains_get_property,
	.psy_dsc.set_property 	= ln8411_mains_set_property,
	.psy_dsc.property_is_writeable = ln8411_mains_is_writeable,
	.get_property		= ln8411_gbms_mains_get_property,
	.set_property 		= ln8411_gbms_mains_set_property,
	.property_is_writeable	= ln8411_gbms_mains_is_writeable,
	.psy_dsc.num_properties	= ARRAY_SIZE(ln8411_mains_properties),
	.forward		= true,
};

#if IS_ENABLED(CONFIG_OF)
static int of_ln8411_dt(struct device *dev,
			 struct ln8411_platform_data *pdata)
{
	struct device_node *np_ln8411 = dev->of_node;
	int ret;

	if(!np_ln8411)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_ln8411, "ln8411,irq-gpio", 0);
	dev_info(dev, "irq-gpio: %d \n", pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_ln8411, "ln8411,input-current-limit",
				   &pdata->iin_cfg_max);
	if (ret) {
		dev_warn(dev, "ln8411,input-current-limit is Empty\n");
		pdata->iin_cfg_max = LN8411_IIN_CFG_DFT;
	}
	pdata->iin_cfg = pdata->iin_cfg_max;
	dev_info(dev, "ln8411,iin_cfg is %u\n", pdata->iin_cfg);

	/* TA max voltage limit */
	ret = of_property_read_u32(np_ln8411, "ln8411,ta-max-vol-4_1",
				   &pdata->ta_max_vol_4_1);
	if (ret) {
		dev_warn(dev, "ln8411,ta-max-vol-4_1 is Empty\n");
		pdata->ta_max_vol_4_1 = LN8411_TA_MAX_VOL;
	}
	ret = of_property_read_u32(np_ln8411, "ln8411,ta-max-vol-2_1",
				   &pdata->ta_max_vol_2_1);
	if (ret) {
		dev_warn(dev, "ln8411,ta-max-vol_2_1 is Empty\n");
		pdata->ta_max_vol_2_1 = LN8411_TA_MAX_VOL_2_1;
	}

	/* input topoff current */
	ret = of_property_read_u32(np_ln8411, "ln8411,input-itopoff",
				   &pdata->iin_topoff);
	if (ret) {
		dev_warn(dev, "ln8411,input-itopoff is Empty\n");
		pdata->iin_topoff = LN8411_IIN_DONE_DFT;
	}
	dev_info(dev, "ln8411,iin_topoff is %u\n", pdata->iin_topoff);

	/* iin offsets */
	ret = of_property_read_u32(np_ln8411, "ln8411,iin-max-offset",
				   &pdata->iin_max_offset);
	if (ret)
		pdata->iin_max_offset = LN8411_IIN_MAX_OFFSET;
	dev_info(dev, "ln8411,iin_max_offset is %u\n", pdata->iin_max_offset);

	ret = of_property_read_u32(np_ln8411, "ln8411,iin-cc_comp-offset",
				   &pdata->iin_cc_comp_offset);
	if (ret)
		pdata->iin_cc_comp_offset = LN8411_IIN_CC_COMP_OFFSET;
	dev_info(dev, "ln8411,iin_cc_comp_offset is %u\n", pdata->iin_cc_comp_offset);

	pdata->si_fet_ovp_drive = of_property_read_bool(np_ln8411, "ln8411,si-fet-ovp-drive");
	dev_info(dev, "ln8411,si-fet-ovp-drive is %d\n", pdata->si_fet_ovp_drive);

#if IS_ENABLED(CONFIG_THERMAL)
	/* USBC thermal zone */
	ret = of_property_read_string(np_ln8411, "google,usb-port-tz-name",
				      &pdata->usb_tz_name);
	if (ret) {
		dev_info(dev, "google,usb-port-tz-name is Empty\n");
		pdata->usb_tz_name = NULL;
	} else {
		dev_info(dev, "google,usb-port-tz-name is %s\n", pdata->usb_tz_name);
	}
#endif

	return 0;
}
#else
static int of_ln8411_dt(struct device *dev,
			 struct ln8411_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_THERMAL)
static int ln8411_usb_tz_read_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct ln8411_charger *ln8411 = tzd->devdata;

	if (!ln8411)
		return -ENODEV;
	*temp = ln8411_read_adc(ln8411, ADCCH_DIETEMP);

	return 0;
}

static struct thermal_zone_device_ops ln8411_usb_tzd_ops = {
	.get_temp = ln8411_usb_tz_read_temp,
};
#endif

static int debug_apply_offsets(void *data, u64 val)
{
	struct ln8411_charger *chip = data;
	int ret;

	ret = ln8411_set_new_cc_max(chip, chip->cc_max);
	dev_info(chip->dev, "Apply offsets iin_max_o=%d iin_cc_comp_o=%d ret=%d\n",
		chip->pdata->iin_max_offset, chip->pdata->iin_cc_comp_offset,
		ret);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(apply_offsets_debug_ops, NULL, debug_apply_offsets, "%#02llx\n");


static int debug_adc_chan_get(void *data, u64 *val)
{
	struct ln8411_charger *ln8411 = data;

	*val = ln8411_read_adc(data, ln8411->debug_adc_channel);
	return 0;
}

static int debug_adc_chan_set(void *data, u64 val)
{
	struct ln8411_charger *ln8411 = data;

	if (val < ADCCH_IIN || val >= ADCCH_MAX)
		return -EINVAL;
	ln8411->debug_adc_channel = val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_adc_chan_ops, debug_adc_chan_get,
			debug_adc_chan_set, "%llu\n");

static int debug_ftm_mode_get(void *data, u64 *val)
{
	struct ln8411_charger *ln8411 = data;
	*val = ln8411->ftm_mode;
	return 0;
}

static int debug_ftm_mode_set(void *data, u64 val)
{
	struct ln8411_charger *ln8411 = data;

	if (val) {
		ln8411->ftm_mode = true;
		ln8411->ta_type = TA_TYPE_USBPD;
		ln8411->chg_mode = CHG_4TO1_DC_MODE;
	} else {
		ln8411->ftm_mode = false;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(debug_ftm_mode_ops, debug_ftm_mode_get, debug_ftm_mode_set, "%llu\n");

static int debug_pps_index_get(void *data, u64 *val)
{
	struct ln8411_charger *ln8411 = data;

	*val = ln8411->pps_index;
	return 0;
}

static int debug_pps_index_set(void *data, u64 val)
{
	struct ln8411_charger *ln8411 = data;

	return ln8411_set_charging_enabled(ln8411, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_pps_index_ops, debug_pps_index_get,
			debug_pps_index_set, "%llu\n");

static ssize_t chg_stats_show(struct device *dev, struct device_attribute *attr,
				    char *buff)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ln8411_charger *ln8411 = i2c_get_clientdata(client);
	struct ln8411_chg_stats *chg_data = &ln8411->chg_data;
	const int max_size = PAGE_SIZE;
	int len = -ENODATA;

	mutex_lock(&ln8411->lock);

	if (!ln8411_chg_stats_valid(chg_data))
		goto exit_done;

	len = scnprintf(buff, max_size,
			"D:%#x,%#x %#x,%#x,%#x,%#x,%#x\n",
			chg_data->adapter_capabilities[0],
			chg_data->adapter_capabilities[1],
			chg_data->receiver_state[0],
			chg_data->receiver_state[1],
			chg_data->receiver_state[2],
			chg_data->receiver_state[3],
			chg_data->receiver_state[4]);
	len += scnprintf(&buff[len], max_size - len,
			"N: ovc=%d,ovc_ibatt=%d,ovc_delta=%d rcp=%d,stby=%d\n",
			chg_data->ovc_count, chg_data->ovc_max_ibatt, chg_data->ovc_max_delta,
			chg_data->rcp_count,
			chg_data->stby_count);
	len += scnprintf(&buff[len], max_size - len,
			"C: nc=%d,pre=%d,ca=%d,cc=%d,cv=%d,adj=%d\n",
			chg_data->nc_count,
			chg_data->pre_count,
			chg_data->ca_count,
			chg_data->cc_count,
			chg_data->cv_count,
			chg_data->adj_count);

exit_done:
	mutex_unlock(&ln8411->lock);
	return len;
}

static ssize_t chg_stats_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ln8411_charger *ln8411 = i2c_get_clientdata(client);

	mutex_lock(&ln8411->lock);
	ln8411_chg_stats_init(&ln8411->chg_data);
	mutex_unlock(&ln8411->lock);

	return count;
}

static DEVICE_ATTR_RW(chg_stats);

static ssize_t registers_dump_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct ln8411_charger *ln8411 = dev_get_drvdata(dev);
	u8 tmp[0x9c - LN8411_DEVICE_ID + 1];
	int ret = 0, i;
	int len = 0;

	ret = regmap_bulk_read(ln8411->regmap, LN8411_DEVICE_ID, &tmp, sizeof(tmp));
	if (ret < 0)
		return ret;

	for (i = 0; i < sizeof(tmp); i++)
		len += scnprintf(&buf[len], PAGE_SIZE - len, "%02x: %02x\n", i, tmp[i]);

	return len;
}

static DEVICE_ATTR_RO(registers_dump);

static ssize_t soft_reset_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ln8411_charger *ln8411 = dev_get_drvdata(dev);
	int ret;
	ln8411->hw_init_done = false;
	ret = ln8411_hw_init(ln8411);
	if (!ret)
		ln8411->hw_init_done = true;

	return count;
}
static DEVICE_ATTR_WO(soft_reset);

static int ln8411_create_fs_entries(struct ln8411_charger *chip)
{

	device_create_file(chip->dev, &dev_attr_chg_stats);
	device_create_file(chip->dev, &dev_attr_registers_dump);
	device_create_file(chip->dev, &dev_attr_soft_reset);

	chip->debug_root = debugfs_create_dir("charger-ln8411", NULL);
	if (IS_ERR_OR_NULL(chip->debug_root)) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -ENOENT;
	}

	debugfs_create_bool("wlc_rampout_iin", 0644, chip->debug_root,
			     &chip->wlc_ramp_out_iin);
	debugfs_create_u32("wlc_rampout_delay", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_delay);
	debugfs_create_u32("wlc_rampout_vout_target", 0644, chip->debug_root,
			   &chip->wlc_ramp_out_vout_target);


	debugfs_create_u32("debug_level", 0644, chip->debug_root,
			   &debug_printk_prlog);
	debugfs_create_u32("no_logbuffer", 0644, chip->debug_root,
			   &debug_no_logbuffer);

	debugfs_create_file("data", 0644, chip->debug_root, chip, &register_debug_ops_ln8411);
	debugfs_create_x32("address", 0644, chip->debug_root, &chip->debug_address);

	debugfs_create_file("1_2_mode", 0664, chip->debug_root, chip, &ln8411_1_2_mode_ops);
	debugfs_create_u32("iin_max_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_max_offset);
	debugfs_create_u32("iin_cc_comp_offset", 0644, chip->debug_root,
			   &chip->pdata->iin_cc_comp_offset);
	debugfs_create_file("apply_offsets", 0644, chip->debug_root, chip,
			    &apply_offsets_debug_ops);

	chip->debug_adc_channel = ADCCH_VOUT;
	debugfs_create_file("adc_chan", 0644, chip->debug_root, chip,
			    &debug_adc_chan_ops);
	debugfs_create_file("pps_index", 0644, chip->debug_root, chip,
			    &debug_pps_index_ops);
	debugfs_create_file("ftm_mode", 0644, chip->debug_root, chip,
			    &debug_ftm_mode_ops);

	return 0;
}


static int ln8411_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	static char *battery[] = { "ln8411-battery" };
	struct power_supply_config mains_cfg = {};
	struct ln8411_platform_data *pdata;
	struct ln8411_charger *ln8411_charger;
	struct device *dev = &client->dev;
	const char *psy_name = NULL;
	int ret;

	dev_dbg(dev, "%s: =========START=========\n", __func__);

	ln8411_charger = devm_kzalloc(dev, sizeof(*ln8411_charger), GFP_KERNEL);
	if (!ln8411_charger)
		return -ENOMEM;

#if IS_ENABLED(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ln8411_platform_data),
				     GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		ret = of_ln8411_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, ln8411_charger);
	ln8411_charger->client = client;

	ln8411_charger->regmap = devm_regmap_init_i2c(client, &ln8411_regmap);
	if (IS_ERR(ln8411_charger->regmap)) {
		dev_err(&client->dev, "ERROR: Cannot probe i2c!\n");
		return -EINVAL;
	}

	ret = get_chip_info(ln8411_charger);
	if (ret) {
		dev_err(&client->dev, "ERROR: Cannot read chip info!\n");
		return -ENODEV;
	}

	mutex_init(&ln8411_charger->lock);
	ln8411_charger->dev = &client->dev;
	ln8411_charger->pdata = pdata;
	ln8411_charger->charging_state = DC_STATE_NO_CHARGING;
	ln8411_charger->wlc_ramp_out_iin = true;
	ln8411_charger->wlc_ramp_out_vout_target = 15300000; /* 15.3V as default */
	ln8411_charger->wlc_ramp_out_delay = 300; /* 300 ms default */
	ln8411_charger->hw_init_done = false;
	ln8411_charger->ibus_ucp_retry_cnt = LN8411_MAX_IBUS_UCP_RETRY_CNT;
	ln8411_charger->ibus_ucp_debounce_cnt = LN8411_MAX_IBUS_UCP_DEBOUNCE_COUNT;
	ln8411_charger->low_batt_retry_cnt = LN8411_MAX_LOW_BATT_RETRY_CNT;

	/* Create a work queue for the direct charger */
	ln8411_charger->dc_wq = alloc_ordered_workqueue("ln8411_dc_wq", WQ_MEM_RECLAIM);
	if (ln8411_charger->dc_wq == NULL) {
		dev_err(ln8411_charger->dev, "failed to create work queue\n");
		mutex_destroy(&ln8411_charger->lock);
		return -ENOMEM;
	}

	ln8411_charger->monitor_wake_lock =
		wakeup_source_register(NULL, "ln8411-charger-monitor");
	if (!ln8411_charger->monitor_wake_lock) {
		dev_err(dev, "Failed to register wakeup source\n");
		destroy_workqueue(ln8411_charger->dc_wq);
		mutex_destroy(&ln8411_charger->lock);
		return -ENODEV;
	}

	/* initialize work */
	INIT_DELAYED_WORK(&ln8411_charger->timer_work, ln8411_timer_work);
	ln8411_charger->timer_id = TIMER_ID_NONE;
	ln8411_charger->timer_period = 0;

	INIT_DELAYED_WORK(&ln8411_charger->pps_work, ln8411_pps_request_work);
	INIT_DELAYED_WORK(&ln8411_charger->init_hw_work, ln8411_init_hw_work);
	ret = of_property_read_string(dev->of_node,
				      "ln8411,psy_name", &psy_name);

	ret = ln8411_probe_pps(ln8411_charger);
	if (ret < 0) {
		dev_warn(dev, "ln8411: PPS not available (%d)\n", ret);
	} else {
		const char *logname = "ln8411";

		ln8411_charger->log = logbuffer_register(logname);
		if (IS_ERR(ln8411_charger->log)) {
			dev_err(dev, "no logbuffer (%ld)\n", PTR_ERR(ln8411_charger->log));
			ln8411_charger->log = NULL;
		}
	}

	schedule_delayed_work(&ln8411_charger->init_hw_work, 0);
	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = ln8411_charger;
	ln8411_charger->mains = devm_power_supply_register(dev,
							&ln8411_mains_desc.psy_dsc,
							&mains_cfg);
	if (IS_ERR(ln8411_charger->mains)) {
		ret = -ENODEV;
		goto error;
	}

	ln8411_charger->attrs.attrs = ln8411_attr_group;
	ret = ln8411_create_fs_entries(ln8411_charger);
	if (ret < 0)
		dev_err(dev, "error while registering debugfs %d\n", ret);

#if IS_ENABLED(CONFIG_GPIOLIB)
	ln8411_gpio_init(ln8411_charger);
	ln8411_charger->gpio.parent = &client->dev;
	ln8411_charger->gpio.of_node = of_find_node_by_name(client->dev.of_node,
							    ln8411_charger->gpio.label);
	if (!ln8411_charger->gpio.of_node)
		dev_err(&client->dev, "Failed to find %s DT node\n", ln8411_charger->gpio.label);

	ret = devm_gpiochip_add_data(&client->dev, &ln8411_charger->gpio, ln8411_charger);
	dev_info(&client->dev, "%d GPIOs registered ret: %d\n", ln8411_charger->gpio.ngpio, ret);
#endif

#if IS_ENABLED(CONFIG_THERMAL)
	if (pdata->usb_tz_name) {
		ln8411_charger->usb_tzd =
			thermal_zone_device_register(pdata->usb_tz_name, 0, 0,
						     ln8411_charger,
						     &ln8411_usb_tzd_ops,
						     NULL, 0, 0);
		if (IS_ERR(ln8411_charger->usb_tzd)) {
			ln8411_charger->usb_tzd = NULL;
			ret = PTR_ERR(ln8411_charger->usb_tzd);
			dev_err(dev, "Couldn't register usb connector thermal zone ret=%d\n",
				ret);
		}
	}
#endif

	ln8411_charger->dc_avail = NULL;
	ln8411_charger->init_done = true;
	dev_info(dev, "ln8411: probe_done\n");
	return 0;

error:
	destroy_workqueue(ln8411_charger->dc_wq);
	mutex_destroy(&ln8411_charger->lock);
	wakeup_source_unregister(ln8411_charger->monitor_wake_lock);
	return ret;
}

static void ln8411_remove(struct i2c_client *client)
{
	struct ln8411_charger *ln8411_charger = i2c_get_clientdata(client);

	/* stop charging if it is active */
	ln8411_stop_charging(ln8411_charger);

	if (client->irq) {
		free_irq(client->irq, ln8411_charger);
		gpio_free(ln8411_charger->pdata->irq_gpio);
	}

	destroy_workqueue(ln8411_charger->dc_wq);

	wakeup_source_unregister(ln8411_charger->monitor_wake_lock);

#if IS_ENABLED(CONFIG_THERMAL)
	if (ln8411_charger->usb_tzd)
		thermal_zone_device_unregister(ln8411_charger->usb_tzd);
#endif
	if (ln8411_charger->log)
		logbuffer_unregister(ln8411_charger->log);
	pps_free(&ln8411_charger->pps_data);
}

static const struct i2c_device_id ln8411_id[] = {
	{ "LN8411", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ln8411_id);

#if IS_ENABLED(CONFIG_OF)
static struct of_device_id ln8411_dt_ids[] = {
	{ .compatible = "ln8411",},
	{ },
};

MODULE_DEVICE_TABLE(of, ln8411_dt_ids);
#endif /* CONFIG_OF */

#if IS_ENABLED(CONFIG_PM)
#if IS_ENABLED(CONFIG_RTC_HCTOSYS)
static int get_current_time(struct ln8411_charger *ln8411, unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		dev_err(ln8411->dev, "%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		dev_err(ln8411->dev, "Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		dev_err(ln8411->dev, "Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	*now_tm_sec = rtc_tm_to_time64(&tm);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static void
ln8411_check_and_update_charging_timer(struct ln8411_charger *ln8411)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(ln8411, &current_time);

	if (ln8411->timer_id != TIMER_ID_NONE)	{
		next_update_time = ln8411->last_update_time +
				(ln8411->timer_period / 1000); /* seconds */

		dev_dbg(ln8411->dev, "%s: current_time=%ld, next_update_time=%ld\n",
			__func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&ln8411->lock);
		ln8411->timer_period = time_left * 1000; /* ms unit */
		mutex_unlock(&ln8411->lock);
		schedule_delayed_work(&ln8411->timer_work,
				msecs_to_jiffies(ln8411->timer_period));

		dev_dbg(ln8411->dev, "%s: timer_id=%d, time_period=%ld\n", __func__,
			 ln8411->timer_id, ln8411->timer_period);
	}
	ln8411->last_update_time = current_time;
}
#endif

static int ln8411_suspend(struct device *dev)
{
	struct ln8411_charger *ln8411 = dev_get_drvdata(dev);

	dev_dbg(ln8411->dev, "%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&ln8411->timer_work);
	return 0;
}

static int ln8411_resume(struct device *dev)
{
	struct ln8411_charger *ln8411 = dev_get_drvdata(dev);

	dev_dbg(ln8411->dev, "%s: update_timer\n", __func__);

	/* Update the current timer */
#if IS_ENABLED(CONFIG_RTC_HCTOSYS)
	ln8411_check_and_update_charging_timer(ln8411);
#else
	if (ln8411->timer_id != TIMER_ID_NONE) {
		mutex_lock(&ln8411->lock);
		ln8411->timer_period = 0;	/* ms unit */
		mutex_unlock(&ln8411->lock);
		schedule_delayed_work(&ln8411->timer_work,
				      msecs_to_jiffies(ln8411->timer_period));
	}
#endif
	return 0;
}
#else
#define ln8411_suspend		NULL
#define ln8411_resume		NULL
#endif

static const struct dev_pm_ops ln8411_pm_ops = {
	.suspend = ln8411_suspend,
	.resume = ln8411_resume,
};

static struct i2c_driver ln8411_driver = {
	.driver = {
		.name = "LN8411",
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = ln8411_dt_ids,
#endif /* CONFIG_OF */
#if IS_ENABLED(CONFIG_PM)
		.pm = &ln8411_pm_ops,
#endif
	},
	.probe        = ln8411_probe,
	.remove       = ln8411_remove,
	.id_table     = ln8411_id,


};

module_i2c_driver(ln8411_driver);

MODULE_AUTHOR("Prasanna Prapancham <prapancham@google.com>");
MODULE_DESCRIPTION("LN8411 Charger Pump Driver");
MODULE_LICENSE("GPL");
