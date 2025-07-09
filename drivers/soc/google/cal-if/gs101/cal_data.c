#include <soc/google/pmucal_common.h>
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include <soc/google/pmucal_system.h>
#include "../pmucal_powermode.h"

#include "flexpmu_cal_cpu_gs101.h"
#include "flexpmu_cal_local_gs101.h"
#include "flexpmu_cal_p2vmap_gs101.h"
#include "flexpmu_cal_system_gs101.h"
#include "flexpmu_cal_powermode_gs101.h"
#include "flexpmu_cal_define_gs101.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"
#include "cmu-pmu_map.h"

#include "clkout_gs101.c"

#include "acpm_dvfs_gs101.h"

#include "asv_gs101.h"

#include "../ra.h"

#include <soc/google/cmu_ewf.h>

extern unsigned int fin_hz_var;
void __iomem *gpio_alive;

#define GPIO_ALIVE_BASE		(0x174d0000)
#define GPA1_DAT		(0x24)

struct cmu_pmu cmu_pmu_map[] = {
	{0x1A000000, "pd-aoc"},
	{0x1F000000, "pd-bus1"},
	{0x20000000, "pd-bus2"},
	{0x17000000, "pd-eh"},
	{0x1C400000, "pd-g3d"},
	{0x1C400000, "pd-embedded_g3d"},
	{0x11000000, "pd-hsi0"},
	{0x14400000, "pd-hsi2"},
	{0x1C200000, "pd-disp"},
	{0x1C000000, "pd-dpu"},
	{0x1C600000, "pd-g2d"},
	{0x1C800000, "pd-mfc"},
	{0x1A400000, "pd-csis"},
	{0x1AA00000, "pd-pdp"},
	{0x1B400000, "pd-itp"},
	{0x1B000000, "pd-dns"},
	{0x1A800000, "pd-g3aa"},
	{0x1AC00000, "pd-ipp"},
	{0x1B700000, "pd-mcsc"},
	{0x1BA00000, "pd-gdc"},
	{0x1BC00000, "pd-tnr"},
	{0x1CA00000, "pd-bo"},
	{0x1CC00000, "pd-tpu"},
};

void gs101_cal_data_init(void)
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
}

void (*cal_data_init)(void) = gs101_cal_data_init;
int (*wa_set_cmuewf)(unsigned int index, unsigned int en, void *cmu_cmu, int *ewf_refcnt) = NULL;
void (*cal_set_cmu_smpl_warn)(void) = NULL;

char *gs101_get_pd_name_by_cmu(unsigned int addr)
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

char *(*cal_get_pd_name_by_cmu)(unsigned int addr) = gs101_get_pd_name_by_cmu;
