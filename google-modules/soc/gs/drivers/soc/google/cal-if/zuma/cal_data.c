// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *
 */

#include <soc/google/pmucal_common.h>
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include <soc/google/pmucal_system.h>
#include "../pmucal_powermode.h"

#include "flexpmu_cal_cpu_zuma.h"
#include "flexpmu_cal_local_zuma.h"
#include "flexpmu_cal_p2vmap_zuma.h"
#include "flexpmu_cal_system_zuma.h"
#include "flexpmu_cal_define_zuma.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"
#include "cmu-pmu_map.h"

#include "clkout_zuma.c"

#include "acpm_dvfs_zuma.h"

#include "asv_zuma.h"
#include "../ra.h"

#include <soc/google/cmu_ewf.h>

extern unsigned int fin_hz_var;

struct cmu_pmu cmu_pmu_map[] = {
	/* CMU base addr, power domain */
	{0x1EE00000, "pd-g3d"},
	{0x1EE00000, "pd-embedded_g3d"},
	{0x19400000, "pd-dpub"},
	{0x19800000, "pd-dpuf0"},
	{0x19C00000, "pd-dpuf1"},
	{0x1A800000, "pd-g2d"},
	{0x1D400000, "pd-gdc"},
	{0x1D800000, "pd-gse"},
	{0x1D000000, "pd-mcsc"},
	{0x1A400000, "pd-mfc"},
	{0x1C400000, "pd-rgbp"},
	{0x1CC00000, "pd-tnr"},
	{0x19000000, "pd-aoc"},
	{0x20A00000, "pd-aur"},
	{0x1A600000, "pd-bw"},
	{0x17000000, "pd-eh"},
	{0x16C00000, "pd-yuvp"},
	{0x11000000, "pd-hsi0"},
	{0x12000000, "pd-hsi1"},
	{0x1C000000, "pd-ispfe"},
	{0x1A300000, "pd-tpu"},
};

void zuma_cal_data_init(void)
{
	pr_info("%s: cal data init\n", __func__);

	/* cpu inform sfr initialize */
	pmucal_sys_powermode[SYS_SICD] = CPU_INFORM_SICD;
	pmucal_sys_powermode[SYS_SLEEP] = CPU_INFORM_SLEEP;
	pmucal_sys_powermode[SYS_SLEEP_SLCMON] = CPU_INFORM_SLEEP_SLCMON;
	pmucal_sys_powermode[SYS_SLEEP_HSI1ON] = CPU_INFORM_SLEEP_HSI1ON;

	cpu_inform_c2 = CPU_INFORM_C2;
	cpu_inform_cpd = CPU_INFORM_CPD;

	/* TODO: add a logic to cover FIN_HZ_26MHZ */
	fin_hz_var = FIN_HZ_24P576M;
}

void (*cal_data_init)(void) = zuma_cal_data_init;
int (*wa_set_cmuewf)(unsigned int index, unsigned int en, void *cmu_cmu, int *ewf_refcnt) = NULL;
void (*cal_set_cmu_smpl_warn)(void) = NULL;

char *zuma_get_pd_name_by_cmu(unsigned int addr)
{
	int i, map_size;

	map_size = ARRAY_SIZE(cmu_pmu_map);
	for (i = 0; i < map_size; i++) {
		if (cmu_pmu_map[i].cmu == addr)
			break;
	}

	if (i < map_size)
		return cmu_pmu_map[i].pmu;

	return NULL;
}

char *(*cal_get_pd_name_by_cmu)(unsigned int addr) = zuma_get_pd_name_by_cmu;
