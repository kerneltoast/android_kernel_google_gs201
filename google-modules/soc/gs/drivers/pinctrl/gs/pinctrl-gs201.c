// SPDX-License-Identifier: GPL-2.0+
//
// GS specific support for Samsung pinctrl/gpiolib driver
// with eint support.
//
// Copyright (c) 2012 Samsung Electronics Co., Ltd.
//		http://www.samsung.com
// Copyright (c) 2012 Linaro Ltd
//		http://www.linaro.org
// Copyright (c) 2017 Krzysztof Kozlowski <krzk@kernel.org>
//
// This file contains the GS201 specific information required by the
// the Samsung pinctrl/gpiolib driver. It also includes the implementation of
// external gpio and wakeup interrupt support.

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>

#include "pinctrl-samsung.h"
#include "pinctrl-exynos.h"

#define EXYNOS9_PIN_BANK_EINTN(types, pins, reg, id)	\
	{						\
		.type		= &types,		\
		.pctl_offset	= reg,			\
		.nr_pins	= pins,			\
		.eint_type	= EINT_TYPE_NONE,	\
		.name		= id			\
	}

#define EXYNOS9_PIN_BANK_EINTG(types, pins, reg, id, offs, fltcon_offs)	\
	{						\
		.type		= &types,		\
		.pctl_offset	= reg,			\
		.nr_pins	= pins,			\
		.eint_type	= EINT_TYPE_GPIO,	\
		.eint_offset	= offs,			\
		.fltcon_offset	= fltcon_offs,		\
		.name		= id			\
	}

#define EXYNOS9_PIN_BANK_EINTW(types, pins, reg, id, offs, fltcon_offs, eint_n)	\
	{						\
		.type		= &types,		\
		.pctl_offset	= reg,			\
		.nr_pins	= pins,			\
		.eint_type	= EINT_TYPE_WKUP,	\
		.eint_offset	= offs,			\
		.eint_num	= eint_n,		\
		.fltcon_offset	= fltcon_offs,		\
		.name		= id			\
	}

/* bank type for non-alive type
 * (CON bit field: 4, DAT bit field: 1, PUD bit field: 4, DRV bit field: 4)
 * (CONPDN bit field: 2, PUDPDN bit field: 4)
 */
static struct samsung_pin_bank_type bank_type_6  = {
	.fld_width = { 4, 1, 4, 4, 2, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, },
};

/* bank type for alive type
 * (CON bit field: 4, DAT bit field: 1, PUD bit field: 4, DRV bit field: 4)
 */
static struct samsung_pin_bank_type bank_type_7 = {
	.fld_width = { 4, 1, 4, 4, },
	.reg_offset = { 0x00, 0x04, 0x08, 0x0c, },
};

/* pin banks of gs201 pin-controller (ALIVE) */
static struct samsung_pin_bank_data gs201_pin_alive[] = {
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 8, 0x0, "gpa0", 0x00, 0x00, 0),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 7, 0x20, "gpa1", 0x04, 0x08, 8),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 4, 0x40, "gpa2", 0x08, 0x10, 15),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 4, 0x60, "gpa3", 0x0c, 0x14, 20),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 4, 0x80, "gpa4", 0x10, 0x18, 24),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 7, 0xa0, "gpa5", 0x14, 0x1c, 28),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 8, 0xc0, "gpa9", 0x18, 0x24, 35),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 2, 0xe0, "gpa10", 0x1c, 0x2c, 43),
};

/* pin banks of gs201 pin-controller (FAR_ALIVE) */
static struct samsung_pin_bank_data gs201_pin_far_alive[] = {
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 8, 0x0, "gpa6", 0x00, 0x00, 45),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 4, 0x20, "gpa7", 0x04, 0x08, 53),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 8, 0x40, "gpa8", 0x08, 0x0c, 57),
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 2, 0x60, "gpa11", 0x0c, 0x14, 65),
};

/* pin banks of gs201 pin-controller (GSACORE) */
static struct samsung_pin_bank_data gs201_pin_gsacore[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x0, "gps0", 0x00, 0x00),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 8, 0x20, "gps1", 0x04, 0x04),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 3, 0x40, "gps2", 0x08, 0x0c),
};

/* pin banks of gs201 pin-controller (GSACTRL) */
static struct samsung_pin_bank_data gs201_pin_gsactrl[] = {
	EXYNOS9_PIN_BANK_EINTW(bank_type_7, 6, 0x0, "gps3", 0x00, 0x00, 0xFF),
};

/* pin banks of gs201 pin-controller (PERIC0) */
static struct samsung_pin_bank_data gs201_pin_peric0[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 5, 0x0, "gpp0", 0x00, 0x00),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x20, "gpp1", 0x04, 0x08),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x40, "gpp2", 0x08, 0x0c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x60, "gpp3", 0x0c, 0x10),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x80, "gpp4", 0x10, 0x14),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0xa0, "gpp5", 0x14, 0x18),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0xc0, "gpp6", 0x18, 0x1c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0xe0, "gpp7", 0x1c, 0x20),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x100, "gpp8", 0x20, 0x24),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x120, "gpp9", 0x24, 0x28),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x140, "gpp10", 0x28, 0x2c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x160, "gpp11", 0x2c, 0x30),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x180, "gpp12", 0x30, 0x34),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x1a0, "gpp13", 0x34, 0x38),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x1c0, "gpp14", 0x38, 0x3c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x1e0, "gpp15", 0x3c, 0x40),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x200, "gpp16", 0x40, 0x44),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x220, "gpp17", 0x44, 0x48),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x240, "gpp18", 0x48, 0x4c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 5, 0x260, "gpp19", 0x4c, 0x50),
};

/* pin banks of gs201 pin-controller (PERIC1) */
static struct samsung_pin_bank_data gs201_pin_peric1[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 8, 0x0, "gpp20", 0x00, 0x00),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x20, "gpp21", 0x04, 0x08),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x40, "gpp22", 0x08, 0x0c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 8, 0x60, "gpp23", 0x0c, 0x10),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0x80, "gpp24", 0x10, 0x18),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0xa0, "gpp25", 0x14, 0x1c),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 5, 0xc0, "gpp26", 0x18, 0x20),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 4, 0xe0, "gpp27", 0x1c, 0x28),
};

/* pin banks of gs201 pin-controller (HSI1) */
static struct samsung_pin_bank_data gs201_pin_hsi1[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 6, 0x0, "gph0", 0x00, 0x00),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 7, 0x20, "gph1", 0x04, 0x08),
};

/* pin banks of gs201 pin-controller (HSI2) */
static struct samsung_pin_bank_data gs201_pin_hsi2[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 6, 0x0, "gph2", 0x00, 0x00),
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 6, 0x20, "gph4", 0x04, 0x08),
};

/* pin banks of gs201 pin-controller (HSI2UFS) */
static struct samsung_pin_bank_data gs201_pin_hsi2ufs[] = {
	EXYNOS9_PIN_BANK_EINTG(bank_type_6, 2, 0x0, "gph3", 0x00, 0x00),
};

static const struct samsung_pin_ctrl gs201_pin_ctrl[] __initconst = {
	{
		/* pin banks of gs201 pin-controller (ALIVE) */
		.pin_banks	= gs201_pin_alive,
		.nr_banks	= ARRAY_SIZE(gs201_pin_alive),
		.eint_gpio_init = exynos_eint_gpio_init,
		.eint_wkup_init = exynos_eint_wkup_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (FAR_ALIVE) */
		.pin_banks	= gs201_pin_far_alive,
		.nr_banks	= ARRAY_SIZE(gs201_pin_far_alive),
		.eint_gpio_init = exynos_eint_gpio_init,
		.eint_wkup_init = exynos_eint_wkup_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (GSACORE) */
		.pin_banks	= gs201_pin_gsacore,
		.nr_banks	= ARRAY_SIZE(gs201_pin_gsacore),
		.eint_gpio_init = exynos_eint_gpio_init,
	}, {
		/* pin banks of gs201 pin-controller (GSACTRL) */
		.pin_banks	= gs201_pin_gsactrl,
		.nr_banks	= ARRAY_SIZE(gs201_pin_gsactrl),
		.eint_gpio_init = exynos_eint_gpio_init,
	}, {
		/* pin banks of gs201 pin-controller (PERIC0) */
		.pin_banks	= gs201_pin_peric0,
		.nr_banks	= ARRAY_SIZE(gs201_pin_peric0),
		.eint_gpio_init = exynos_eint_gpio_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (PERIC1) */
		.pin_banks	= gs201_pin_peric1,
		.nr_banks	= ARRAY_SIZE(gs201_pin_peric1),
		.eint_gpio_init = exynos_eint_gpio_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume	= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (HSI1) */
		.pin_banks	= gs201_pin_hsi1,
		.nr_banks	= ARRAY_SIZE(gs201_pin_hsi1),
		.eint_gpio_init = exynos_eint_gpio_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (HSI2) */
		.pin_banks	= gs201_pin_hsi2,
		.nr_banks	= ARRAY_SIZE(gs201_pin_hsi2),
		.eint_gpio_init = exynos_eint_gpio_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	}, {
		/* pin banks of gs201 pin-controller (HSI2UFS) */
		.pin_banks	= gs201_pin_hsi2ufs,
		.nr_banks	= ARRAY_SIZE(gs201_pin_hsi2ufs),
		.eint_gpio_init = exynos_eint_gpio_init,
		.suspend	= exynos_pinctrl_suspend,
		.resume		= exynos_pinctrl_resume,
	},
};

const struct samsung_pinctrl_of_match_data gs201_of_data __initconst = {
	.ctrl		= gs201_pin_ctrl,
	.num_ctrl	= ARRAY_SIZE(gs201_pin_ctrl),
};
