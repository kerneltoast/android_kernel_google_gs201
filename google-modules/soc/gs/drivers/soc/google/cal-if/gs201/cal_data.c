#include <soc/google/pmucal_common.h>
#include <soc/google/pmucal_system.h>
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include "../pmucal_powermode.h"

#include "flexpmu_cal_cpu_gs201.h"
#include "flexpmu_cal_local_gs201.h"
#include "flexpmu_cal_p2vmap_gs201.h"
#include "flexpmu_cal_system_gs201.h"
#include "flexpmu_cal_define_gs201.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"
#include "cmu-pmu_map.h"

#include "clkout_gs201.c"

#include "acpm_dvfs_gs201.h"

#include "asv_gs201.c"

#include "../ra.h"

#include <soc/google/cmu_ewf.h>

extern unsigned int fin_hz_var;
void __iomem *gpio_alive;

#define GPIO_ALIVE_BASE		(0x180D0000)
#define GPA1_DAT		(0x24)

struct cmu_pmu cmu_pmu_map[] = {
	/* TODO: will be added after bring-up done */
};

void gs201_cal_data_init(void)
{
	pr_info("%s: cal data init\n", __func__);

	/* cpu inform sfr initialize */
	pmucal_sys_powermode[SYS_SICD] = CPU_INFORM_SICD;
	pmucal_sys_powermode[SYS_SLEEP] = CPU_INFORM_SLEEP;
	pmucal_sys_powermode[SYS_SLEEP_SLCMON] = CPU_INFORM_SLEEP_SLCMON;
	pmucal_sys_powermode[SYS_SLEEP_HSI1ON] = CPU_INFORM_SLEEP_HSI1ON;

	cpu_inform_c2 = CPU_INFORM_C2;
	cpu_inform_cpd = CPU_INFORM_CPD;

	gpio_alive = ioremap(GPIO_ALIVE_BASE, SZ_4K);
	if (!gpio_alive) {
		pr_err("%s: gpio_alive ioremap failed\n", __func__);
		BUG();
	}

	/* check DEBUG_SEL and determine FIN src */
	if (__raw_readl(gpio_alive + GPA1_DAT) & (1 << 6))
		fin_hz_var = FIN_HZ_26M;
	else
		fin_hz_var = 24576000;

	cmucal_dbg_mux_dbg_offset(0x4004);

	asv_init();
}

void (*cal_data_init)(void) = gs201_cal_data_init;
int (*wa_set_cmuewf)(unsigned int index, unsigned int en, void *cmu_cmu, int *ewf_refcnt) = NULL;
void (*cal_set_cmu_smpl_warn)(void) = NULL;

char *gs201_get_pd_name_by_cmu(unsigned int addr)
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

char *(*cal_get_pd_name_by_cmu)(unsigned int addr) = gs201_get_pd_name_by_cmu;
