/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_EL3_MON_H
#define __EXYNOS_EL3_MON_H

/* Error code */
#define EXYNOS_ERROR_TZASC_WRONG_REGION		(-1)

#define EXYNOS_ERROR_ALREADY_INITIALIZED	(1)
#define EXYNOS_ERROR_NOT_VALID_ADDRESS		(0x1000)

/* Mode for setting TZPC of runtime PM domain */
#define EXYNOS_GET_IN_PD_DOWN			(0)
#define EXYNOS_WAKEUP_PD_DOWN			(1)

#define RUNTIME_PM_TZPC_GROUP			(2)

#define PRIV_REG_OPTION_READ			0
#define PRIV_REG_OPTION_WRITE			1
#define PRIV_REG_OPTION_RMW			2

int exynos_pd_tz_save(unsigned int addr);
int exynos_pd_tz_restore(unsigned int addr);
int set_priv_reg(phys_addr_t reg, u32 val);
int get_priv_reg(phys_addr_t reg);
int rmw_priv_reg(phys_addr_t reg, u32 mask, u32 val);

#endif	/* __EXYNOS_EL3_MON_H */
