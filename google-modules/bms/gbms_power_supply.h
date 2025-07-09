/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GBMS_POWER_SUPPLY_H__
#define __GBMS_POWER_SUPPLY_H__

#include <linux/power_supply.h>

enum {
	GBMS_TAPER_CONTROL_OFF = 0,
	GBMS_TAPER_CONTROL_ON,
};

/* Indicates USB Type-C CC connection status */
/* Deprecated */
enum power_supply_typec_mode {
	POWER_SUPPLY_TYPEC_NONE,

	/* Acting as source */
	POWER_SUPPLY_TYPEC_SINK,		/* Rd only */
	POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE,	/* Rd/Ra */
	POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY,/* Rd/Rd */
	POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER,	/* Ra/Ra */
	POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY,	/* Ra only */

	/* Acting as sink */
	POWER_SUPPLY_TYPEC_SOURCE_DEFAULT,	/* Rp default */
	POWER_SUPPLY_TYPEC_SOURCE_MEDIUM,	/* Rp 1.5A */
	POWER_SUPPLY_TYPEC_SOURCE_HIGH,		/* Rp 3A */
	POWER_SUPPLY_TYPEC_DAM_DEFAULT,		/* Rp-1.5A/Rp-3A */
	POWER_SUPPLY_TYPEC_DAM_MEDIUM,		/* Rp-Default/Rp-1.5A */
	POWER_SUPPLY_TYPEC_DAM_HIGH,		/* Rp-Default/Rp-3A */

	/* Non Compliant */
	POWER_SUPPLY_TYPEC_NON_COMPLIANT,
	POWER_SUPPLY_TYPEC_RP_STD_STD,		/* Rp-Default/Rp-Default */
	POWER_SUPPLY_TYPEC_RP_MED_MED,		/* Rp-1.5A/Rp-1.5A */
	POWER_SUPPLY_TYPEC_RP_HIGH_HIGH,	/* Rp-3A/Rp-3A */
};

/* Deprecated */
enum power_supply_typec_src_rp {
	POWER_SUPPLY_TYPEC_SRC_RP_STD,
	POWER_SUPPLY_TYPEC_SRC_RP_1P5A,
	POWER_SUPPLY_TYPEC_SRC_RP_3A
};

/* Deprecated */
enum power_supply_typec_power_role {
	POWER_SUPPLY_TYPEC_PR_NONE,		/* CC lines in high-Z */
	POWER_SUPPLY_TYPEC_PR_DUAL,
	POWER_SUPPLY_TYPEC_PR_SINK,
	POWER_SUPPLY_TYPEC_PR_SOURCE,
};

enum gbms_property {
	/* I am not proud of this */
	GBMS_PROP_LOCAL_EXTENSIONS = POWER_SUPPLY_PROP_SERIAL_NUMBER + 100,

	GBMS_PROP_ADAPTER_DETAILS,		/* GBMS Adapter Details */
	GBMS_PROP_BATT_CE_CTRL,			/* GBMS plugged (replace with GBMS_PROP_CHARGING_ENABLED) */
	GBMS_PROP_CAPACITY_RAW,			/* GBMS used for ssoc */
	GBMS_PROP_CHARGING_ENABLED,		/* GBMS cpm control */
	GBMS_PROP_CHARGE_CHARGER_STATE,		/* GBMS charge, need uint64 */
	GBMS_PROP_CHARGE_DISABLE,		/* GBMS disconnect */
	GBMS_PROP_DEAD_BATTERY,			/* GBMS during boot */
	GBMS_PROP_INPUT_CURRENT_LIMITED,	/* can be device prop */
	GBMS_PROP_TAPER_CONTROL,		/* GBMS DC, needs for last tier */
	GBMS_PROP_HEALTH_ACT_IMPEDANCE,		/* GBMS activation impedance, qualified */
	GBMS_PROP_HEALTH_IMPEDANCE,		/* GBMS impedance, qualified */
	GBMS_PROP_RESISTANCE,			/* GBMS battery resistance, unqualified */
	GBMS_PROP_RESISTANCE_RAW,		/* GBMS battery resistance, unqualified, u16 */
	GBMS_PROP_RESISTANCE_AVG,		/* GBMS google_resistance */
	GBMS_PROP_BATTERY_AGE,			/* GBMS time in field */
	GBMS_PROP_CAPACITY_FADE_RATE,		/* GBMS capacity fade rate by fullcapnom */
	GBMS_PROP_CAPACITY_FADE_RATE_FCR,	/* GBMS capacity fade rate by fullcaprep */
	GBMS_PROP_CHARGE_FULL_ESTIMATE,		/* GBMS google_capacity */
	GBMS_PROP_WLC_OP_FREQ,			/* GBMS wlc frequency */
	GBMS_PROP_WLC_VRECT,			/* GBMS wlc Vrect */
	GBMS_PROP_FG_REG_LOGGING,		/* GBMS FG logging */
	GBMS_PROP_WLC_VCPOUT,			/* GBMS wlc cpout voltage */
	GBMS_PROP_BATT_ID,			/* GBMS battery id */
	GBMS_PROP_RECAL_FG,			/* GBMS FG reset */
	GBMS_PROP_LOGBUFFER_BD,			/* GBMS pass logbuffer_bd address */
	GBMS_PROP_AAFV,				/* GBMS pass aafv to FG */
};

union gbms_propval {
	union power_supply_propval prop;
	int64_t int64val;
};

#define gbms_propval_int64val(psp) \
	container_of(psp, union gbms_propval, prop)->int64val

/* GBMS properties -------------------------------------------------------- */

struct gbms_desc {
	struct power_supply_desc psy_dsc;
	bool forward;

	/* bgms properties */
	int (*get_property)(struct power_supply *psy, enum gbms_property psp,
			    union gbms_propval *val);
	int (*set_property)(struct power_supply *psy, enum gbms_property psp,
			    const union gbms_propval *val);
	int (*property_is_writeable)(struct power_supply *psy,
				     enum gbms_property psp);
};

extern int gbms_set_property(struct power_supply *psy, enum gbms_property psp,
			     const union gbms_propval *val);
extern int gbms_get_property(struct power_supply *psy, enum gbms_property psp,
			     union gbms_propval *val);

#endif /* __GBMS_POWER_SUPPLY_H__ */
