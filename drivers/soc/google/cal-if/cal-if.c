#include <linux/module.h>
#if 0
#include <linux/debug-snapshot.h>
#endif
#include <soc/google/ect_parser.h>
#include <soc/google/cal-if.h>
#ifdef CONFIG_EXYNOS9820_BTS
#include <soc/google/bts.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG)
#include <soc/google/exynos-bcm_dbg.h>
#endif

#include <trace/events/power.h>

#include "pwrcal-env.h"
#include "pwrcal-rae.h"
#include "cmucal.h"
#include "ra.h"
#include "acpm_dvfs.h"
#include "fvmap.h"
#include "asv.h"

#include "pmucal_system.h"
#include "pmucal_local.h"
#include "pmucal_cpu.h"
#include "pmucal_dbg.h"
#include "pmucal_cp.h"
#include "pmucal_rae.h"
#include "pmucal_powermode.h"

#include "../acpm/acpm.h"

extern s32 gs_chipid_get_dvfs_version(void);

int (*exynos_cal_pd_bcm_sync)(unsigned int id, bool on);
EXPORT_SYMBOL(exynos_cal_pd_bcm_sync);

static DEFINE_SPINLOCK(pmucal_cpu_lock);

unsigned int cal_clk_is_enabled(unsigned int id)
{
	return 0;
}
EXPORT_SYMBOL_GPL(cal_clk_is_enabled);

unsigned long cal_dfs_get_max_freq(unsigned int id)
{
	return vclk_get_max_freq(id);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_max_freq);

unsigned long cal_dfs_get_min_freq(unsigned int id)
{
	return vclk_get_min_freq(id);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_min_freq);

unsigned int cal_dfs_get_lv_num(unsigned int id)
{
	return vclk_get_lv_num(id);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_lv_num);

int cal_dfs_set_rate(unsigned int id, unsigned long rate)
{
	struct vclk *vclk;
	int ret;

	if (IS_ACPM_VCLK(id)) {
		ret = exynos_acpm_set_rate(GET_IDX(id), rate);
		if (!ret) {
			vclk = cmucal_get_node(id);
			if (vclk)
				vclk->vrate = (unsigned int)rate;
		}
	} else {
		ret = vclk_set_rate(id, rate);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_set_rate);

int cal_dfs_set_rate_switch(unsigned int id, unsigned long switch_rate)
{
	int ret = 0;

	ret = vclk_set_rate_switch(id, switch_rate);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_set_rate_switch);

int cal_dfs_set_rate_restore(unsigned int id, unsigned long switch_rate)
{
	int ret = 0;

	ret = vclk_set_rate_restore(id, switch_rate);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_set_rate_restore);

unsigned long cal_dfs_cached_get_rate(unsigned int id)
{
	unsigned long ret;

	ret = vclk_get_rate(id);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_cached_get_rate);

unsigned long cal_dfs_get_rate(unsigned int id)
{
	unsigned long ret;

	ret = vclk_recalc_rate(id);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_get_rate);

int cal_dfs_get_rate_table(unsigned int id, unsigned long *table)
{
	int ret;

	ret = vclk_get_rate_table(id, table);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_dfs_get_rate_table);

int cal_clk_setrate(unsigned int id, unsigned long rate)
{
	int ret = -EINVAL;

	ret = vclk_set_rate(id, rate);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_clk_setrate);

unsigned long cal_clk_getrate(unsigned int id)
{
	unsigned long ret = 0;

	ret = vclk_recalc_rate(id);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_clk_getrate);

int cal_clk_enable(unsigned int id)
{
	int ret = 0;

	ret = vclk_set_enable(id);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_clk_enable);

int cal_clk_disable(unsigned int id)
{
	int ret = 0;

	ret = vclk_set_disable(id);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_clk_disable);

int cal_qch_init(unsigned int id, unsigned int use_qch)
{
	int ret = 0;

	ret = ra_set_qch(id, use_qch, 0, 0);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_qch_init);

unsigned int cal_dfs_get_boot_freq(unsigned int id)
{
	return vclk_get_boot_freq(id);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_boot_freq);

unsigned int cal_dfs_get_resume_freq(unsigned int id)
{
	return vclk_get_resume_freq(id);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_resume_freq);

int cal_pd_control(unsigned int id, int on)
{
	unsigned int index;
	int ret;

	if ((id & 0xFFFF0000) != BLKPWR_MAGIC)
		return -1;

	index = id & 0x0000FFFF;

	if (on) {
		ret = pmucal_local_enable(index);
#ifdef CONFIG_EXYNOS9820_BTS
		if (index == 0x7)
			bts_pd_sync(id, on);
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG)
		if (exynos_cal_pd_bcm_sync && cal_pd_status(id))
			exynos_cal_pd_bcm_sync(id, true);
#endif
	} else {
#ifdef CONFIG_EXYNOS9820_BTS
		if (index == 0x7)
			bts_pd_sync(id, on);
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG)
		if (exynos_cal_pd_bcm_sync && cal_pd_status(id))
			exynos_cal_pd_bcm_sync(id, false);
#endif
		ret = pmucal_local_disable(index);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(cal_pd_control);

int cal_pd_status(unsigned int id)
{
	unsigned int index;

	if ((id & 0xFFFF0000) != BLKPWR_MAGIC)
		return -1;

	index = id & 0x0000FFFF;

	return pmucal_local_is_enabled(index);
}
EXPORT_SYMBOL_GPL(cal_pd_status);

int cal_pd_set_smc_id(unsigned int id, int need_smc)
{
	unsigned int index;

	if (need_smc && ((id & 0xFFFF0000) != BLKPWR_MAGIC))
		return -1;

	index = id & 0x0000FFFF;

	pmucal_local_set_smc_id(index, need_smc);

	return 0;
}
EXPORT_SYMBOL_GPL(cal_pd_set_smc_id);

int cal_pm_enter(int mode)
{
	char clock_name[32] = {0};
	scnprintf(clock_name, 32, "CAL_PM_ENTER_%d", mode);
	trace_clock_set_rate(clock_name, 1, raw_smp_processor_id());
	return pmucal_system_enter(mode);
}
EXPORT_SYMBOL_GPL(cal_pm_enter);

int cal_pm_exit(int mode)
{
	char clock_name[32] = {0};
	scnprintf(clock_name, 32, "CAL_PM_ENTER_%d", mode);
	trace_clock_set_rate(clock_name, 0, raw_smp_processor_id());
	return pmucal_system_exit(mode);
}
EXPORT_SYMBOL_GPL(cal_pm_exit);

int cal_pm_earlywakeup(int mode)
{
	char clock_name[32] = {0};
	scnprintf(clock_name, 32, "CAL_PM_ENTER_%d", mode);
	trace_clock_set_rate(clock_name, 0, raw_smp_processor_id());
	return pmucal_system_earlywakeup(mode);
}
EXPORT_SYMBOL_GPL(cal_pm_earlywakeup);

int cal_cpu_enable(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_enable(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cpu_enable);

int cal_cpu_disable(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_disable(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cpu_disable);

int cal_cpu_status(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_is_enabled(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cpu_status);

int cal_cluster_enable(unsigned int cluster)
{
	int ret;
	char clock_name[32] = {0};

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_enable(cluster);
	spin_unlock(&pmucal_cpu_lock);

	scnprintf(clock_name, 32, "CAL_CLUSTER_ENABLE_%u", cluster);
	trace_clock_set_rate(clock_name, 1, raw_smp_processor_id());

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cluster_enable);

int cal_cluster_disable(unsigned int cluster)
{
	int ret;
	char clock_name[32] = {0};

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_disable(cluster);
	spin_unlock(&pmucal_cpu_lock);


	scnprintf(clock_name, 32, "CAL_CLUSTER_ENABLE_%u", cluster);
	trace_clock_set_rate(clock_name, 0, raw_smp_processor_id());

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cluster_disable);

int cal_cluster_status(unsigned int cluster)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_is_enabled(cluster);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cluster_status);

int cal_cluster_req_emulation(unsigned int cluster, bool en)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_req_emulation(cluster, en);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(cal_cluster_req_emulation);

extern int cal_is_lastcore_detecting(unsigned int cpu)
{
	return pmucal_is_lastcore_detecting(cpu);
}
EXPORT_SYMBOL_GPL(cal_is_lastcore_detecting);

int cal_dfs_get_asv_table(unsigned int id, unsigned int *table)
{
	return fvmap_get_voltage_table(id, table);
}
EXPORT_SYMBOL_GPL(cal_dfs_get_asv_table);

void cal_dfs_set_volt_margin(unsigned int id, int volt)
{
	if (IS_ACPM_VCLK(id))
		exynos_acpm_set_volt_margin(id, volt);
}
EXPORT_SYMBOL_GPL(cal_dfs_set_volt_margin);

int cal_dfs_get_rate_asv_table(unsigned int id,
					struct dvfs_rate_volt *table)
{
	unsigned long rate[48];
	unsigned int volt[48];
	int num_of_entry;
	int idx;

	num_of_entry = cal_dfs_get_rate_table(id, rate);
	if (num_of_entry == 0)
		return 0;

	if (num_of_entry != cal_dfs_get_asv_table(id, volt))
		return 0;

	for (idx = 0; idx < num_of_entry; idx++) {
		table[idx].rate = rate[idx];
		table[idx].volt = volt[idx];
	}

	return num_of_entry;
}
EXPORT_SYMBOL_GPL(cal_dfs_get_rate_asv_table);

int cal_asv_get_ids_info(unsigned int id)
{
	return asv_get_ids_info(id);
}
EXPORT_SYMBOL_GPL(cal_asv_get_ids_info);

int cal_asv_get_grp(unsigned int id)
{
	return asv_get_grp(id);
}
EXPORT_SYMBOL_GPL(cal_asv_get_grp);

#if IS_ENABLED(CONFIG_CP_PMUCAL)
int cal_cp_init(void)
{
	return pmucal_cp_init();
}
EXPORT_SYMBOL_GPL(cal_cp_init);

int cal_cp_status(void)
{
	return pmucal_cp_status();
}
EXPORT_SYMBOL_GPL(cal_cp_status);

int cal_cp_reset_assert(void)
{
	return pmucal_cp_reset_assert();
}
EXPORT_SYMBOL_GPL(cal_cp_reset_assert);

int cal_cp_reset_release(void)
{
	return pmucal_cp_reset_release();
}
EXPORT_SYMBOL_GPL(cal_cp_reset_release);

void cal_cp_active_clear(void)
{
	pmucal_cp_active_clear();
}
EXPORT_SYMBOL_GPL(cal_cp_active_clear);

void cal_cp_reset_req_clear(void)
{
	pmucal_cp_reset_req_clear();
}
EXPORT_SYMBOL_GPL(cal_cp_reset_req_clear);

void cal_cp_enable_dump_pc_no_pg(void)
{
	pmucal_cp_enable_dump_pc_no_pg();
}
EXPORT_SYMBOL_GPL(cal_cp_enable_dump_pc_no_pg);

void cal_cp_disable_dump_pc_no_pg(void)
{
	pmucal_cp_disable_dump_pc_no_pg();
}
EXPORT_SYMBOL_GPL(cal_cp_disable_dump_pc_no_pg);
#endif

int cal_if_init(void *np)
{
	static int cal_initialized;
	struct resource res;
	int ret, len;
	const __be32 *prop;
	unsigned int minmax_idx = 0;

	if (cal_initialized == 1)
		return 0;

	prop = of_get_property(np, "minmax_idx", &len);
	if (prop) {
		minmax_idx = be32_to_cpup(prop);
	} else {
		int dvfs_version = gs_chipid_get_dvfs_version();
		if (dvfs_version < 0)
			return dvfs_version;
		minmax_idx = dvfs_version;
	}

	ect_parse_binary_header();

	vclk_initialize(minmax_idx);

	if (cal_data_init)
		cal_data_init();

	ret = pmucal_rae_init();
	if (ret < 0)
		return ret;

	ret = pmucal_system_init();
	if (ret < 0)
		return ret;

	ret = pmucal_local_init();
	if (ret < 0)
		return ret;

	ret = pmucal_cpu_init();
	if (ret < 0)
		return ret;

	ret = pmucal_cpuinform_init();
	if (ret < 0)
		return ret;

	pmucal_dbg_init();

#if IS_ENABLED(CONFIG_CP_PMUCAL)
	ret = pmucal_cp_initialize();
	if (ret < 0)
		return ret;
#endif

	exynos_acpm_set_device(np);

	if (of_address_to_resource(np, 0, &res) == 0)
		cmucal_dbg_set_cmu_top_base(res.start);

	cal_initialized = 1;
#ifdef CONFIG_DEBUG_FS
	vclk_debug_init();
#endif
	pmucal_dbg_debugfs_init();

	return 0;
}

static int cal_if_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = cal_if_init(np);
	if (ret)
		goto out;

	ret = fvmap_init(get_fvmap_base());

out:
	return ret;
}

static const struct of_device_id cal_if_match[] = {
	{ .compatible = "samsung,exynos_cal_if" },
	{},
};
MODULE_DEVICE_TABLE(of, cal_if_match);

static struct platform_driver samsung_cal_if_driver = {
	.probe	= cal_if_probe,
	.driver	= {
		.name = "exynos-cal-if",
		.owner	= THIS_MODULE,
		.of_match_table	= cal_if_match,
	},
};

static int exynos_cal_if_init(void)
{
	int ret;

	ret = platform_driver_register(&samsung_cal_if_driver);
	if (ret) {
		pr_err("samsung_cal_if_driver probe failed.\n");
		goto err_out;
	}

	ret = exynos_acpm_dvfs_init();
	if (ret) {
		pr_err("samsung_cal_if_driver probe failed.\n");
		platform_driver_unregister(&samsung_cal_if_driver);
		ret = -EINVAL;
	}

err_out:
	return ret;
}
arch_initcall(exynos_cal_if_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos Chip Abstraction Layer Interface");
