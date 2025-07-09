/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 Google, LLC
 *
 * SW Support for MAX77779 IF-PMIC
 */

#ifndef MAX77779_H_
#define MAX77779_H_

#include <linux/i2c.h>

//#define CONFIG_SCNPRINTF_DEBUG 0
#include "max77779_regs.h"

#define MAX77779_CHG_INT_COUNT 2

#define MAX77779_PMIC_REV_A0		0x01
#define MAX77779_PMIC_REV_A1		0x02

#define MAX77779_PMIC_ID_SEQ	0x79
#define MAX77779_PMIC_OF_NAME	"max77779,pmic"

/* FG's reg 0x40 and status value of 0x82 are not documented */
#define MAX77779_FG_BOOT_CHECK_REG 0x40
#define MAX77779_FG_BOOT_CHECK_SUCCESS 0x82

#define MAX77779_REASON_FIRMWARE        "FW_UPDATE"


struct device* max77779_get_dev(struct device *dev, const char *name);

/* write to a register */
int max77779_external_chg_reg_write(struct device *dev, u8 reg, u8 value);
/* read a register */
int max77779_external_chg_reg_read(struct device *dev, u8 reg, u8 *value);
/* update a register */
int max77779_external_chg_reg_update(struct device *dev, u8 reg, u8 mask, u8 value);
/* change the mode register */
int max77779_external_chg_mode_write(struct device *dev, enum max77779_charger_modes mode);
/* change the insel register */
int max77779_external_chg_insel_write(struct device *dev, u8 mask, u8 value);
/* read the insel register */
int max77779_external_chg_insel_read(struct device *dev, u8 *value);

int max77779_external_pmic_reg_read(struct device *dev, uint8_t reg, uint8_t *val);
int max77779_external_pmic_reg_write(struct device *dev, uint8_t reg, uint8_t val);
int max77779_external_pmic_reg_update(struct device *dev, uint8_t reg, uint8_t msk, uint8_t val);

int max77779_external_fg_reg_read(struct device *dev, uint16_t reg, uint16_t *val);
int max77779_external_fg_reg_write(struct device *dev, uint16_t reg, uint16_t val);

int max77779_external_vimon_reg_read(struct device *dev, uint16_t reg, void *val, int len);
int max77779_external_vimon_reg_write(struct device *dev, uint16_t reg, const void *val, int len);
int max77779_external_vimon_read_buffer(struct device *dev, uint16_t *buff, size_t *count,
					size_t buff_max);
int max77779_external_vimon_enable(struct device *dev, bool enable);

int max77779_fg_enable_firmware_update(struct device *dev, bool enable);

static inline int max77779_read_batt_conn(struct device *dev, int *temp)
{
	return -ENODEV;
}
static inline int max77779_read_usb_temp(struct device *dev, int *temp)
{
	return -ENODEV;
}
static inline int max77779_read_batt_id(struct device *dev, unsigned int *id)
{
	return -ENODEV;
}

/* ----------------------------------------------------------------------------
 * GS101 usecases
 * Platform specific, will need to be moved outside the driver.
 *
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSx	Name
 * ----------------------------------------------------------------------------
 * 1-1	1	0	x	0	IF-PMIC-VBUS	0	0/0	USB_CHG
 * 1-2	2	0	x	0	DC VBUS		0	0/0	USB_DC
 * 2-1	1	0	0	1	IF-PMIC-VBUS	2	0/1	USB_CHG_WLC_TX
 * 2-2	2	0	0	1	DC CHG		2	0/1	USB_DC_WLC_TX
 * 3-1	0	0	1	0	IF-PMIC-WCIN	0	0/0	WLC_RX
 * 3-2	0	0	2	0	DC WCIN		0	0/0	WLC_DC
 * 4-1	0	1	1	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	2	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG 5V		0	0/0	USB_OTG_FRS
 * 6-2	0	0	0	1	0		2	0/1	WLC_TX
 * 7-2	0	1	0	1	MW OTG 5V	2	0/1	USB_OTG_WLC_TX
 * 8	0	0	0	0	0		0	0/0	IDLE
 * ----------------------------------------------------------------------------
 *
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * USB_chg = 0 off, 1 = on, 2 = PPS
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 */
struct max77779_foreach_cb_data {
	struct gvotable_election *el;

	const char *reason;

	int chgr_on;	/* CC_MAX != 0 */
	bool stby_on;	/* on disconnect, mode=0 */
	bool charge_done;

	int chgin_off;	/* input_suspend, mode=0 */
	int wlcin_off;	/* input_suspend, mode=0 */
	int usb_wlc;	/* input_suspend, mode=0 */

	/* wlc_on is the same as wlc_rx */
	bool buck_on;	/* wired power in (chgin_on) from TCPCI */

	bool otg_on;	/* power out, usually external */
	bool frs_on;	/* power out, internal boost */

	bool wlc_rx;	/* charging wireless */
	bool wlc_tx;	/* battery share */

	bool dc_on;	/* DC requested - wired or wireless */

	u8 raw_value;	/* hard override */
	bool use_raw;

	bool fwupdate_on; /* enter firmware update mode */

	bool pogo_vin;	/* power in, pogo */
	bool pogo_vout;	/* power out, pogo */

	u8 reg;

	struct gvotable_election *dc_avail_votable;	/* DC_AVAIL */
};

/* internal system values */
enum {
	/* Charging disabled (go to mode 0) */
	GBMS_CHGR_MODE_STBY_ON		= 0x10 + MAX77779_CHGR_MODE_ALL_OFF,
	/* USB inflow off */
	GBMS_CHGR_MODE_CHGIN_OFF	= 0x11 + MAX77779_CHGR_MODE_ALL_OFF,
	/* WCIN inflow off */
	GBMS_CHGR_MODE_WLCIN_OFF	= 0x12 + MAX77779_CHGR_MODE_ALL_OFF,
	/* USB + WLC_RX mode */
	GBMS_CHGR_MODE_USB_WLC_RX	= 0x13 + MAX77779_CHGR_MODE_ALL_OFF,

	/* charging enabled (charging current != 0) */
	GBMS_CHGR_MODE_CHGR_BUCK_ON	= 0x10 + MAX77779_CHGR_MODE_CHGR_BUCK_ON,

	/* boost mode (0x9) during firmware update */
	GBMS_CHGR_MODE_FWUPDATE_BOOST_ON = 0x20 + MAX77779_CHGR_MODE_BOOST_ON,
};


#endif
