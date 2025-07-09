/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 */

#ifndef MAX77779_USECASE_H_
#define MAX77779_USECASE_H_

#define MAX77779_CHG_CNFG_05_WCSM_ILIM_1400_MA 0xA
#define MAX77779_CHG_TX_RETRIES 10

struct max77779_usecase_data {
	int bst_on;		/* ext boost */
	int ext_bst_mode;	/* ext boost mode */
	int otg_enable;		/* enter/exit from OTG cases */
	int ext_bst_ctl;	/* SEQ VENDOR_EXTBST.EXT_BST_EN */
	bool rx_otg_en;		/* enable WLC_RX -> WLC_RX + OTG case */
	bool ext_otg_only;	/* use external OTG only */
	int dc_sw_gpio;		/* WLC-DC switch enable */
	int pogo_vout_en;	/* pogo 5V vout */

	int vin_is_valid;	/* MAX20339 STATUS1.vinvalid */

	int wlc_en;		/* wlcrx/chgin coex */
	int wlc_vbus_en;	/* b/202526678 */
	bool reverse12_en;	/* reverse 1:2 mode */
	int wlc_spoof_gpio;	/* wlcrx thermal throttle */
	u32 wlc_spoof_vbyp;	/* wlc spoof VBYP */

	u8 otg_ilim;		/* TODO: TCPM to control this? */
	u8 otg_vbyp;		/* TODO: TCPM to control this? */
	u8 otg_orig;		/* restore value */
	u8 otg_value;		/* CHG_CNFG_11:VBYPSET for USB OTG Voltage */
	int input_uv;

	struct device *dev;
	bool init_done;
	int use_case;

	int rtx_ready; /* rtx ready gpio from wireless */
	int rtx_available; /* rtx supported gpio from wlc, usecase set for UI */

	struct power_supply *psy;

	bool dcin_is_dock;

	struct gvotable_election *force_5v_votable;
};

enum gsu_usecases {
	GSU_RAW_MODE 		= -1,	/* raw mode, default, */

	GSU_MODE_STANDBY	= 0,	/* 8, PMIC mode 0 */
	GSU_MODE_USB_CHG	= 1,	/* 1-1 wired mode 0x4, mode 0x5 */
	GSU_MODE_USB_DC 	= 2,	/* 1-2 wired mode 0x0 */
	GSU_MODE_USB_CHG_WLC_TX = 3,	/* 2-1, 1041, */

	GSU_MODE_WLC_RX		= 5,	/* 3-1, mode 0x4, mode 0x5 */
	GSU_MODE_WLC_DC		= 6,	/* 3-2, mode 0x0 */

	GSU_MODE_USB_OTG_WLC_RX = 7,	/* 7, 524, */
	GSU_MODE_USB_OTG 	= 9,	/* 5-1, 516,*/
	GSU_MODE_USB_OTG_FRS	= 10,

	GSU_MODE_WLC_TX 	= 11,	/* 6-2, 1056, */
	GSU_MODE_USB_OTG_WLC_TX = 12,
	GSU_MODE_USB_WLC_RX	= 13,

	GSU_MODE_DOCK		= 14,
	GSU_MODE_POGO_VOUT	= 15,
	GSU_MODE_USB_CHG_POGO_VOUT	= 16,
	GSU_MODE_USB_OTG_POGO_VOUT	= 17,

	GSU_MODE_FWUPDATE	= 18,   /* boost mode for frimware update */

	GSU_MODE_STANDBY_BUCK_ON = 100,	/* indicates input_suspend and plugged in */
};

enum wlc_state_t {
	WLC_DISABLED = 0,
	WLC_ENABLED = 1,
	WLC_SPOOFED = 2,
};

extern int gs201_wlc_en(struct max77779_usecase_data *uc_data, enum wlc_state_t state);
extern int gs201_to_standby(struct max77779_usecase_data *uc_data, int use_case);
extern int gs201_to_usecase(struct max77779_usecase_data *uc_data, int use_case);
extern int gs201_finish_usecase(struct max77779_usecase_data *uc_data, int use_case);
extern int gs201_force_standby(struct max77779_usecase_data *uc_data);
extern bool gs201_setup_usecases(struct max77779_usecase_data *uc_data,
				 struct device_node *node);
extern void gs201_dump_usecasase_config(struct max77779_usecase_data *uc_data);
extern int max77779_otg_vbyp_mv_to_code(u8 *code, int vbyp);

#endif
