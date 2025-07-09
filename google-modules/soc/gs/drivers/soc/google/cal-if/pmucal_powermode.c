// SPDX-License-Identifier: GPL-2.0
/*
 * PMUCAL powermode support.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <soc/google/cal-if.h>
#include <soc/google/pwrcal-env.h>
#include <soc/google/pmucal_system.h>
#include "pmucal_powermode.h"
#include "pmucal_rae.h"
#include "pmucal_cpu.h"

void pmucal_powermode_hint(unsigned int mode)
{
	unsigned int cpu = smp_processor_id();

	set_priv_reg(pmucal_cpuinform_list[cpu].base_pa + pmucal_cpuinform_list[cpu].offset, mode);
}

u32 pmucal_get_powermode_hint(unsigned int cpu)
{
	return __raw_readl(pmucal_cpuinform_list[cpu].base_va
			+ pmucal_cpuinform_list[cpu].offset);
}

u32 pmucal_is_lastcore_detecting(unsigned int cpu)
{
	u32 power_mode;

	if (cpu >= cpu_inform_list_size)
		return 0;

	power_mode = pmucal_get_powermode_hint(cpu);

	if (!cal_cpu_status(cpu))
		return 0;

	if (power_mode == cpu_inform_cpd || power_mode == pmucal_sys_powermode[SYS_SICD])
		return 1;

	return 0;
}

void pmucal_powermode_hint_clear(void)
{
	unsigned int cpu = smp_processor_id();

	set_priv_reg(pmucal_cpuinform_list[cpu].base_pa + pmucal_cpuinform_list[cpu].offset, 0);
}

int pmucal_cpuinform_init(void)
{
	int i, j;

	for (i = 0; i < cpu_inform_list_size; i++) {
		for (j = 0; j < pmucal_p2v_list_size; j++)
			if (pmucal_p2v_list[j].pa == (phys_addr_t)pmucal_cpuinform_list[i].base_pa)
				break;

		if (j != pmucal_p2v_list_size) {
			pmucal_cpuinform_list[i].base_va = pmucal_p2v_list[j].va;
		} else {
			pr_err("%s %s: there is no such PA in p2v_list (idx:%d)\n",
					PMUCAL_PREFIX, __func__, i);
			return -ENOENT;
		}

	}

	return 0;
}
