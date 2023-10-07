/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google, LLC
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

#ifndef MAX1730X_H_
#define MAX1730X_H_

#include <linux/device.h>
#include <linux/regmap.h>
#include "max1720x.h"

#define MAX1730X_N_OF_HISTORY_PAGES 100

#define MAX1730X_GAUGE_PASS1	0x404
#define MAX1730X_NVPRTTH1_CHARGING	0x0008
#define MAX1730X_NPROTCFG_PASS1		0x6EA3
#define MAX1730X_NPROTCFG_PASS2		0x0A04

enum max1730x_register {
	MAX1730X_MAXMINVOLT = 0x08,
	MAX1730X_MAXMINTEMP = 0x09,
	MAX1730X_MAXMINCURR = 0x0A,
	MAX1730X_CONFIG = 0x0B,
	MAX1730X_FULLCAPREP = 0x10,
	MAX1730X_VCELL = 0x1A,
	MAX1730X_TEMP = 0x1B,
	MAX1730X_CURRENT = 0x1C,
	MAX1730X_AVGCURRENT = 0x1D,
	MAX1730X_MIXCAP = 0x2B,
	MAX1730X_FULLCAP = 0x35,
	MAX1730X_LEARNCFG = 0xA1,
	MAX1730X_MAXPEAKPWR = 0xA4,
	MAX1730X_SUSPEAKPWR = 0xA5,
	MAX1730X_PACKRESISTANCE = 0xA6,
	MAX1730X_SYSRESISTANCE = 0xA7,
	MAX1730X_MINSYSVOLTAGE = 0xA8,
	MAX1730X_MPPCURRENT = 0xA9,
	MAX1730X_SPPCURRENT = 0xAA,
	MAX1730X_CONFIG2 = 0xAB,
	MAX1730X_IALRTTH = 0xAC,
	MAX1730X_MINVOLT = 0xAD,
	MAX1730X_MINCURR = 0xAE,
	MAX1730X_NVPRTTH1BAK = 0xD6,
	MAX1730X_NPROTCFG = 0xD7,

};

enum max1730x_nvram {
	MAX1730X_NVRAM_START 	= 0x80,
	MAX1730X_NMANFCTRNAME0	= 0xCC,
	MAX1730X_NMANFCTRNAME1	= 0xCD,
	MAX1730X_NVPRTTH1 	= 0xD0,
	MAX1730X_NDPLIMIT	= 0xE0,
	MAX1730X_NSCOCVLIM	= 0xE1,

	MAX1730X_NVRAM_END 	= 0xEF,
	MAX1730X_HISTORY_START 	= 0xF0,
	MAX1730X_HISTORY_WRITE_STATUS_START = 0xF2,
	MAX1730X_HISTORY_VALID_STATUS_END = 0xFB,
	MAX1730X_HISTORY_WRITE_STATUS_END = 0xFE,
	MAX1730X_HISTORY_END	= 0xFF,
};

enum max1730x_command_bits {
	MAX1730X_COMMAND_FUEL_GAUGE_RESET = 0x8000,
	MAX1730X_READ_HISTORY_CMD_BASE = 0xE22E,
	MAX1730X_COMMAND_HISTORY_RECALL_WRITE_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_0 = 0xE29C,
	MAX1730X_COMMAND_HISTORY_RECALL_VALID_1 = 0xE29D,
};

#define MAX1730X_HISTORY_PAGE_SIZE \
	(MAX1730X_HISTORY_END - MAX1730X_HISTORY_START + 1)

#define MAX1730X_N_OF_HISTORY_FLAGS_REG \
	(MAX1730X_HISTORY_END - \
		MAX1730X_HISTORY_END + 1 + \
		MAX1730X_HISTORY_VALID_STATUS_END - \
		MAX1730X_HISTORY_START + 1)

/** ------------------------------------------------------------------------ */

static bool max1730x_is_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX1720X_COMMAND:
	case MAX1720X_COMMSTAT:
	case MAX1720X_LOCK:
	case MAX1720X_ODSCTH:
	case MAX1720X_ODSCCFG:
	case MAX1720X_VFOCV:
	case MAX1720X_VFSOC:
	case 0x00 ... 0x4F:
	case 0xA0 ... 0xAE:
	case 0xB0 ... 0xDF:
	case 0xF0:
	case 0xF5:
		return true;
	}

	return false;
}

static const struct regmap_config max1730x_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1720X_VFSOC,
	.readable_reg = max1730x_is_reg,
	.volatile_reg = max1730x_is_reg,
};

static bool max1730x_is_nvram_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX1730X_NVRAM_START &&
		reg <= MAX1730X_HISTORY_END);
}

const struct regmap_config max1730x_regmap_nvram_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX1730X_HISTORY_END,
	.readable_reg = max1730x_is_nvram_reg,
	.volatile_reg = max1730x_is_nvram_reg,
};

/** ------------------------------------------------------------------------ */

/* see b/119416045 for layout */
static const struct max17x0x_reg max1730x[] = {
	[MAX17X0X_TAG_avgc] = { ATOM_INIT_REG16(MAX1730X_AVGCURRENT)},
	[MAX17X0X_TAG_cnfg] = { ATOM_INIT_REG16(MAX1730X_CONFIG)},
	[MAX17X0X_TAG_mmdv] = { ATOM_INIT_REG16(MAX1730X_MAXMINVOLT)},
	[MAX17X0X_TAG_vcel] = { ATOM_INIT_REG16(MAX1730X_VCELL)},
	[MAX17X0X_TAG_temp] = { ATOM_INIT_REG16(MAX1730X_TEMP)},
	[MAX17X0X_TAG_curr] = { ATOM_INIT_REG16(MAX1730X_CURRENT)},
	[MAX17X0X_TAG_mcap] = { ATOM_INIT_REG16(MAX1730X_MIXCAP)},
	[MAX17X0X_TAG_avgr] = { ATOM_INIT_REG16(MAX1730X_NMANFCTRNAME1) },
	[MAX17X0X_TAG_vfsoc] = { ATOM_INIT_REG16(MAX1720X_VFSOC)},
	[MAX17X0X_TAG_vfocv] = { ATOM_INIT_REG16(MAX1720X_VFOCV)},

	[MAX17X0X_TAG_BCNT] = { ATOM_INIT_MAP(0x8e, 0x8f, 0x9d, 0x9e, 0x9f,
					      0xb2, 0xb4, 0xb6, 0xc7, 0xe2)},
	[MAX17X0X_TAG_SNUM] = { ATOM_INIT_MAP(0xce, 0xe6, 0xe7, 0xe8, 0xe9,
					      0xea, 0xeb, 0xec, 0xed, 0xee,
					      0xef) },

	[MAX17X0X_TAG_HSTY] = { ATOM_INIT_SET(0xf0, 0xf2, 0xfb, 0xfe, 0xff) },
	[MAX17X0X_TAG_BCEA] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME0,
					      MAX1730X_NDPLIMIT,
					      MAX1730X_NSCOCVLIM) },
	[MAX17X0X_TAG_rset] = { ATOM_INIT_SET16(MAX1730X_CONFIG2,
					MAX1730X_COMMAND_FUEL_GAUGE_RESET,
					700)},
	[MAX17X0X_TAG_BRES] = { ATOM_INIT_SET(MAX1730X_NMANFCTRNAME1,
					      MAX1730X_NMANFCTRNAME0) },
};

#endif