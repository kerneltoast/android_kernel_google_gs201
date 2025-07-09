// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *            http://www.samsung.com/
 *
 * EXYNOS - EL3 monitor power domain save/restore support
 * Author: Jang Hyunsung <hs79.jang@samsung.com>
 *
 */

#include <linux/arm-smccc.h>
#include <linux/module.h>
#include <soc/google/exynos-el3_mon.h>
#include <linux/soc/samsung/exynos-smc.h>

int exynos_pd_tz_save(unsigned int addr)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PREPARE_PD_ONOFF,
		      EXYNOS_GET_IN_PD_DOWN,
		      addr,
		      RUNTIME_PM_TZPC_GROUP, 0, 0, 0, 0, &res);
	return (int)res.a0;
}
EXPORT_SYMBOL(exynos_pd_tz_save);

int exynos_pd_tz_restore(unsigned int addr)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PREPARE_PD_ONOFF,
		      EXYNOS_WAKEUP_PD_DOWN,
		      addr,
		      RUNTIME_PM_TZPC_GROUP, 0, 0, 0, 0, &res);
	return (int)res.a0;
}
EXPORT_SYMBOL(exynos_pd_tz_restore);

int set_priv_reg(phys_addr_t reg, u32 val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG,
		      reg,
		      PRIV_REG_OPTION_WRITE,
		      val, 0, 0, 0, 0, &res);

	return (int)res.a0;
}
EXPORT_SYMBOL(set_priv_reg);

int get_priv_reg(phys_addr_t reg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG,
		      reg,
		      PRIV_REG_OPTION_READ,
		      0, 0, 0, 0, 0, &res);

	return (int)res.a0;
}
EXPORT_SYMBOL_GPL(get_priv_reg);

int rmw_priv_reg(phys_addr_t reg, u32 mask, u32 val)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PRIV_REG,
		      reg,
		      PRIV_REG_OPTION_RMW,
		      mask, val, 0, 0, 0, &res);

	return (int)res.a0;
}
EXPORT_SYMBOL(rmw_priv_reg);

MODULE_SOFTDEP("pre: exynos-el2");
MODULE_LICENSE("GPL");
