// SPDX-License-Identifier: <GPL-2.0-only>
/*
 * Platform driver and device for retrieving ACPM stats
 *
 * Copyright 2020 Google LLC
 *
 * Author: bsschwar@google.com
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <clocksource/arm_arch_timer.h>
#include <linux/clocksource.h>
#include <linux/list.h>

#include <soc/google/acpm_ipc_ctrl.h>
#include <soc/google/exynos-pd.h>
#include "fw_header/acpm_power_stats.h"

#define GS_POWER_STATS_PREFIX "power_stats: "

#if IS_ENABLED(CONFIG_SOC_GS101)
static char const *const mif_user_names[NUM_MIF_USERS] = { "AOC", "GSA" };
#elif IS_ENABLED(CONFIG_SOC_GS201)
static char const *const mif_user_names[NUM_MIF_USERS] = { "AOC", "GSA", "TPU"};
#else
static char const *const mif_user_names[NUM_MIF_USERS] = { "AOC", "GSA", "TPU", "AUR"};
#endif

static char const *const slc_user_names[NUM_SLC_USERS] = { "AOC" };

static char const *const sys_powermode_names[NUM_SYS_POWERMODE] = {
	"SICD", "SLEEP", "SLEEP_SLCMON", "SLEEP_HSI1ON", "STOP"
};

static char const *const cluster_names[NUM_CLUSTERS] = { "CLUSTER0", "CLUSTER1",
							 "CLUSTER2" };

#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
static char const *const core_names[NUM_CORES] = { "CORE00", "CORE01", "CORE02",
						   "CORE03", "CORE10", "CORE11",
						   "CORE20", "CORE21" };
#else
static char const *const core_names[NUM_CORES] = { "CORE00", "CORE01", "CORE02",
						   "CORE03", "CORE10", "CORE11",
						   "CORE12", "CORE13", "CORE20" };
#endif

#if IS_ENABLED(CONFIG_SOC_GS101)
static char const *const domain_names[NUM_DOMAINS] = { "MIF", "TPU", "CL0",
						       "CL1", "CL2"};
#else
static char const *const domain_names[NUM_DOMAINS] = { "MIF", "TPU", "CL0",
						       "CL1", "CL2", "AUR"};
#endif

struct pd_entry {
	struct list_head entry;
	struct exynos_pm_domain *domain;
};

struct power_stats_device {
	struct device *dev;

	union allstats {
		struct fvp_stats fvp_stats;
		struct core_stats core_stats;
		struct soc_stats soc_stats;
		struct pmu_stats pmu_stats;
		struct latency_stats latency_stats;
	} scratch_buffer[2];

	void __iomem *soc_stats_base;
	unsigned int soc_stats_fails;

	void __iomem *core_stats_base;
	unsigned int core_stats_fails;

	void __iomem *fvp_stats_base;
	unsigned int fvp_stats_fails;

	void __iomem *pmu_stats_base;
	unsigned int pmu_stats_fails;

	void __iomem *latency_stats_base;
	unsigned int latency_stats_fails;

	u32 mult;
	u32 shift;

	struct list_head pd_list;

	struct mutex lock;
};

static inline u64 cyc_to_ns(u64 cyc, u32 mult, u32 shift)
{
	return (cyc * mult) >> shift;
}

/*
 * Read a region of memory until two consecutive reads yield the same results.
 * In order to avoid data race while memory region is being read.
 * Will fail (returns -ETIMEDOUT) after 5 attempts. returns 0 on success
 */
static int safe_memcpy_fromio(void *dst, const volatile void __iomem *src,
			      void *scratch, long size)
{
	int i;

	memcpy_fromio(dst, src, size);
	for (i = 0; i < 5; i++) {
		memcpy_fromio((i & 1) ? dst : scratch, src, size);
		if (0 == memcmp(dst, scratch, size)) {
			return 0;
		}
	}

	pr_err(GS_POWER_STATS_PREFIX "Failed to copy from io, limit reached\n");

	return -ETIMEDOUT;
}

static ssize_t
print_lpm_stats(struct power_stats_device *ps_dev, char *buf, ssize_t size,
		const struct lpm_stat lpm_stats[NUM_SYS_POWERMODE], u64 now)

{
	int i;
	u64 elapsed_time;
	ssize_t s = 0;

	for (i = 0; i < NUM_SYS_POWERMODE; i++) {
		elapsed_time = (lpm_stats[i].last_entry_timestamp >
				lpm_stats[i].last_exit_timestamp) ?
					     now - lpm_stats[i].last_entry_timestamp :
					     0;

		s += scnprintf(buf + s, size - s,
			       "%s\n"
			       "\tsuccess_count: %llu\n"
			       "\twakeup_count: %llu\n"
			       "\terror_count: %llu\n"
			       "\ttotal_time_ns: %llu\n"
			       "\tlast_entry_time_ns: %llu\n"
			       "\tlast_exit_time_ns: %llu\n",
			       sys_powermode_names[i],
			       lpm_stats[i].success_count,
			       lpm_stats[i].early_wakeup_count,
			       lpm_stats[i].error_count,
			       cyc_to_ns(lpm_stats[i].total_time + elapsed_time,
					 ps_dev->mult, ps_dev->shift),
			       cyc_to_ns(lpm_stats[i].last_entry_timestamp,
					 ps_dev->mult, ps_dev->shift),
			       cyc_to_ns(lpm_stats[i].last_exit_timestamp,
					 ps_dev->mult, ps_dev->shift));
	}
	s += scnprintf(buf + s, size - s, "\n");

	return s;
}

static ssize_t print_resource_stats(struct power_stats_device *ps_dev,
				    char *buf, ssize_t size,
				    const struct resource_stat *stats,
				    const char *const *labels, int len, u64 now)
{
	int i;
	u64 elapsed_time;
	ssize_t s = 0;

	for (i = 0; i < len; i++) {
		elapsed_time = (stats[i].last_down_timestamp >
				stats[i].last_up_timestamp) ?
					     now - stats[i].last_down_timestamp :
					     0;

		s += scnprintf(
			buf + s, size - s,
			"%s\n"
			"\tdown_count: %llu\n"
			"\ttotal_down_time_ns: %llu\n"
			"\tlast_down_time_ns: %llu\n"
			"\tlast_up_time_ns: %llu\n",
			labels[i], stats[i].down_count,
			cyc_to_ns(stats[i].total_down_time + elapsed_time,
				  ps_dev->mult, ps_dev->shift),
			cyc_to_ns(stats[i].last_down_timestamp, ps_dev->mult,
				  ps_dev->shift),
			cyc_to_ns(stats[i].last_up_timestamp, ps_dev->mult,
				  ps_dev->shift));
	}
	s += scnprintf(buf + s, size - s, "\n");

	return s;
}

static ssize_t print_resource_req_stats(struct power_stats_device *ps_dev,
					char *buf, ssize_t size,
					const struct resource_req_stat *stats,
					const char *const *labels, int len,
					u64 now)
{
	int i;
	u64 elapsed_time;
	ssize_t s = 0;

	for (i = 0; i < len; i++) {
		elapsed_time = (stats[i].last_req_up_timestamp >
				stats[i].last_req_down_timestamp) ?
					     now - stats[i].last_req_up_timestamp :
					     0;

		s += scnprintf(
			buf + s, size - s,
			"%s\n"
			"\treq_up_count: %llu\n"
			"\ttotal_req_up_time_ns: %llu\n"
			"\tlast_req_up_time_ns: %llu\n"
			"\tlast_req_down_time_ns: %llu\n",
			labels[i], stats[i].req_up_count,
			cyc_to_ns(stats[i].total_req_up_time + elapsed_time,
				  ps_dev->mult, ps_dev->shift),
			cyc_to_ns(stats[i].last_req_up_timestamp, ps_dev->mult,
				  ps_dev->shift),
			cyc_to_ns(stats[i].last_req_down_timestamp,
				  ps_dev->mult, ps_dev->shift));
	}
	s += scnprintf(buf + s, size - s, "\n");

	return s;
}

static ssize_t print_latency_stat(struct power_stats_device *ps_dev, char *buf,
				  ssize_t size, const struct min_max_stat *stat)
{
	ssize_t s = 0;

	s += scnprintf(buf + s, size - s,
		       "\tavg_ns: %llu\n"
		       "\tmin_ns: %llu\n"
		       "\tmax_ns: %llu\n",
		       stat->count == 0 ? 0 :
						cyc_to_ns(stat->total, ps_dev->mult,
						    ps_dev->shift) /
						  stat->count,
		       stat->count == 0 ? ~0 :
						cyc_to_ns(stat->min, ps_dev->mult,
						    ps_dev->shift),
		       cyc_to_ns(stat->max, ps_dev->mult, ps_dev->shift));

	return s;
}

static ssize_t print_histogram(struct power_stats_device *ps_dev, char *buf,
			       ssize_t size, const struct freq_histogram *hist,
			       u64 now)
{
	int i;
	u64 elapsed_time;
	ssize_t s = 0;

	for (i = 0; i < MAX_NUM_FREQS; i++) {
		if (hist->freqs[i] == 0)
			break;

		elapsed_time = (hist->cur_freq == hist->freqs[i]) ?
					     now - hist->freq_change_timestamp :
					     0;

		s += scnprintf(buf + s, size - s,
			       "\t%u: count = %llu time_ns = %llu\n",
			       hist->freqs[i], hist->freq_count[i],
			       cyc_to_ns(hist->freq_time[i] + elapsed_time,
					 ps_dev->mult, ps_dev->shift));
	}
	s += scnprintf(buf + s, size - s, "\tcur_freq: %u\n", hist->cur_freq);
	s += scnprintf(buf + s, size - s, "\tlast_freq_change_time_ns: %llu\n",
		       cyc_to_ns(hist->freq_change_timestamp, ps_dev->mult,
				 ps_dev->shift));

	return s;
}

static ssize_t fvp_stats_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int i;
	ssize_t s = 0;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);
	struct fvp_stats *stats = &(ps_dev->scratch_buffer[0].fvp_stats);
	struct fvp_stats *scratch = &(ps_dev->scratch_buffer[1].fvp_stats);

	if (!ps_dev->fvp_stats_base)
		return 0;

	mutex_lock(&ps_dev->lock);
	if (safe_memcpy_fromio(stats, ps_dev->fvp_stats_base, scratch,
			       sizeof(struct fvp_stats))) {
		ps_dev->fvp_stats_fails++;
	} else {
		u64 now = get_frc_time();

		for (i = 0; i < NUM_DOMAINS; i++) {
			s += scnprintf(buf + s, PAGE_SIZE - s, "%s\n",
				       domain_names[i]);

			s += print_histogram(ps_dev, buf + s, PAGE_SIZE - s,
					     &(stats->freq_hist[i]), now);
		}
	}
	mutex_unlock(&ps_dev->lock);

	return s;
}

static ssize_t soc_stats_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	ssize_t s = 0;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);
	struct soc_stats *stats = &(ps_dev->scratch_buffer[0].soc_stats);
	struct soc_stats *scratch = &(ps_dev->scratch_buffer[1].soc_stats);

	if (!ps_dev->soc_stats_base)
		return 0;

	mutex_lock(&ps_dev->lock);
	if (safe_memcpy_fromio(stats, ps_dev->soc_stats_base, scratch,
			       sizeof(struct soc_stats))) {
		ps_dev->soc_stats_fails++;
	} else {
		u64 now = get_frc_time();

		s += scnprintf(buf, PAGE_SIZE, "LPM:\n");
		s += print_lpm_stats(ps_dev, buf + s, PAGE_SIZE - s,
				     stats->lpm_stats, now);

		s += scnprintf(buf + s, PAGE_SIZE - s, "MIF:\n");
		s += print_resource_stats(ps_dev, buf + s, PAGE_SIZE - s,
					  stats->mif_stats, sys_powermode_names,
					  NUM_SYS_POWERMODE, now);

		s += scnprintf(buf + s, PAGE_SIZE - s, "MIF_REQ:\n");
		s += print_resource_req_stats(ps_dev, buf + s, PAGE_SIZE - s,
					      stats->mif_req_stats,
					      mif_user_names, NUM_MIF_USERS,
					      now);

		s += scnprintf(buf + s, PAGE_SIZE - s, "SLC:\n");
		s += print_resource_stats(ps_dev, buf + s, PAGE_SIZE - s,
					  stats->slc_stats, sys_powermode_names,
					  NUM_SYS_POWERMODE, now);

		s += scnprintf(buf + s, PAGE_SIZE - s, "SLC_REQ:\n");
		s += print_resource_req_stats(ps_dev, buf + s, PAGE_SIZE - s,
					      stats->slc_req_stats,
					      slc_user_names, NUM_SLC_USERS,
					      now);
	}
	mutex_unlock(&ps_dev->lock);

	return s;
}

static ssize_t core_stats_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t s = 0;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);
	struct core_stats *stats = &(ps_dev->scratch_buffer[0].core_stats);
	struct core_stats *scratch = &(ps_dev->scratch_buffer[1].core_stats);

	if (!ps_dev->core_stats_base)
		return 0;

	mutex_lock(&ps_dev->lock);
	if (safe_memcpy_fromio(stats, ps_dev->core_stats_base, scratch,
			       sizeof(struct core_stats))) {
		ps_dev->core_stats_fails++;
	} else {
		u64 now = get_frc_time();

		s += scnprintf(buf, PAGE_SIZE, "CORES:\n");
		s += print_resource_stats(ps_dev, buf + s, PAGE_SIZE - s,
					  stats->cpu_stats, core_names,
					  NUM_CORES, now);

		s += scnprintf(buf + s, PAGE_SIZE - s, "CLUSTERS:\n");
		s += print_resource_stats(ps_dev, buf + s, PAGE_SIZE - s,
					  stats->cluster_stats, cluster_names,
					  NUM_CLUSTERS, now);
	}
	mutex_unlock(&ps_dev->lock);

	return s;
}

static ssize_t pmu_stats_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	ssize_t s = 0;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);
	struct pmu_stats *stats = &(ps_dev->scratch_buffer[0].pmu_stats);
	struct pmu_stats *scratch = &(ps_dev->scratch_buffer[1].pmu_stats);

	if (!ps_dev->pmu_stats_base)
		return 0;

	mutex_lock(&ps_dev->lock);
	if (safe_memcpy_fromio(stats, ps_dev->pmu_stats_base, scratch,
			       sizeof(struct pmu_stats))) {
		ps_dev->pmu_stats_fails++;
	} else {
		u64 now = get_frc_time();

		s += scnprintf(buf + s, PAGE_SIZE - s, "MIF_PWR_REQ:\n");
		s += print_resource_req_stats(ps_dev, buf + s, PAGE_SIZE - s,
					      &(stats->mif_pwr_gsa_req),
					      &(mif_user_names[MIF_USER_GSA]),
					      1, now);
	}
	mutex_unlock(&ps_dev->lock);

	return s;
}

static ssize_t latency_stats_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t s = 0;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);
	struct latency_stats *stats =
		&(ps_dev->scratch_buffer[0].latency_stats);
	struct latency_stats *scratch =
		&(ps_dev->scratch_buffer[1].latency_stats);

	if (!ps_dev->latency_stats_base)
		return 0;

	mutex_lock(&ps_dev->lock);
	if (safe_memcpy_fromio(stats, ps_dev->latency_stats_base, scratch,
			       sizeof(struct latency_stats))) {
		ps_dev->latency_stats_fails++;
	} else {
		s += scnprintf(buf + s, PAGE_SIZE - s, "MIF_DOWN:\n");
		s += print_latency_stat(ps_dev, buf + s, PAGE_SIZE - s,
					&(stats->mif_down));

		s += scnprintf(buf + s, PAGE_SIZE - s, "MIF_UP:\n");
		s += print_latency_stat(ps_dev, buf + s, PAGE_SIZE - s,
					&(stats->mif_up));

		s += scnprintf(buf + s, PAGE_SIZE - s, "SLC_DOWN:\n");
		s += print_latency_stat(ps_dev, buf + s, PAGE_SIZE - s,
					&(stats->slc_down));

		s += scnprintf(buf + s, PAGE_SIZE - s, "SLC_UP:\n");
		s += print_latency_stat(ps_dev, buf + s, PAGE_SIZE - s,
					&(stats->slc_up));
	}
	mutex_unlock(&ps_dev->lock);

	return s;
}

static ssize_t pd_stats_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	ssize_t s = 0;
	struct pd_entry *pd;
	struct exynos_pd_stat stat;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);

	list_for_each_entry (pd, &ps_dev->pd_list, entry) {
		if (exynos_pd_get_pd_stat(pd->domain, &stat)) {
			dev_err(ps_dev->dev, "Failed pd_stat for %s\n",
				pd->domain->genpd.name);
		} else {
			s += scnprintf(buf + s, PAGE_SIZE - s,
				       "%s:\n"
				       "\ton_count: %llu\n"
				       "\ttotal_on_time_ns: %llu\n"
				       "\tlast_on_time_ns: %llu\n"
				       "\tlast_off_time_ns: %llu\n",
				       pd->domain->genpd.name, stat.on_count,
				       ktime_to_ns(stat.total_on_time),
				       ktime_to_ns(stat.last_on_time),
				       ktime_to_ns(stat.last_off_time));
		}
	}
	return s;
}

static ssize_t fail_stats_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t s;
	struct power_stats_device *ps_dev = dev_get_drvdata(dev);

	mutex_lock(&ps_dev->lock);
	s = scnprintf(buf, PAGE_SIZE,
		      "soc_stats_fails:%u\n"
		      "core_stats_fails:%u\n"
		      "fvp_stats_fails:%u\n"
		      "pmu_stats_fails:%u\n"
		      "latency_stats_fails:%u\n",
		      ps_dev->soc_stats_fails, ps_dev->core_stats_fails,
		      ps_dev->fvp_stats_fails, ps_dev->pmu_stats_fails,
		      ps_dev->latency_stats_fails);
	mutex_unlock(&ps_dev->lock);

	return s;
}

static int init_stat_node(struct platform_device *pdev, const char *buffer_name,
			  char **stats_base, u32 stats_size)
{
	int ret = 0;
	u32 buff_size;

	if (acpm_ipc_get_buffer(buffer_name, stats_base, &buff_size) < 0) {
		dev_err(&pdev->dev, "Can not find %s buffer\n", buffer_name);
		ret = -ENOENT;
	} else if (buff_size != stats_size) {
		dev_err(&pdev->dev, "Invalid %s struct\n", buffer_name);
		ret = -ENOENT;
	}

	if (ret)
		*stats_base = NULL;

	return ret;
}

static int init_pd_stat_node(struct power_stats_device *ps_dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct exynos_pm_domain *pd;
	struct pd_entry *new_pd_entry;

	INIT_LIST_HEAD(&ps_dev->pd_list);

	for_each_compatible_node (np, NULL, "samsung,exynos-pd") {
		if (of_device_is_available(np)) {
			pdev = of_find_device_by_node(np);
			pd = (struct exynos_pm_domain *)platform_get_drvdata(
				pdev);

			new_pd_entry = devm_kzalloc(
				ps_dev->dev, sizeof(*new_pd_entry), GFP_KERNEL);
			if (!new_pd_entry)
				return -ENOMEM;

			new_pd_entry->domain = pd;
			list_add(&new_pd_entry->entry, &ps_dev->pd_list);
		}
	}

	return 0;
}

static DEVICE_ATTR_RO(soc_stats);
static DEVICE_ATTR_RO(core_stats);
static DEVICE_ATTR_RO(fvp_stats);
static DEVICE_ATTR_RO(pmu_stats);
static DEVICE_ATTR_RO(latency_stats);
static DEVICE_ATTR_RO(pd_stats);
static DEVICE_ATTR_RO(fail_stats);

static struct attribute *power_stats_attrs[] = {
	&dev_attr_soc_stats.attr,     &dev_attr_core_stats.attr,
	&dev_attr_fvp_stats.attr,     &dev_attr_pmu_stats.attr,
	&dev_attr_latency_stats.attr, &dev_attr_pd_stats.attr,
	&dev_attr_fail_stats.attr,    NULL,
};

ATTRIBUTE_GROUPS(power_stats);

static int power_stats_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 timer_freq_hz;

	struct power_stats_device *ps_dev =
		devm_kzalloc(&pdev->dev, sizeof(*ps_dev), GFP_KERNEL);
	if (!ps_dev)
		return -ENOMEM;

	ps_dev->dev = &pdev->dev;

	ret = of_property_read_u32(pdev->dev.of_node, "timer-frequency-hz",
				   &timer_freq_hz);
	if (ret) {
		dev_err(&pdev->dev, "Error getting timer frequency\n");
		return ret;
	}

	mutex_init(&ps_dev->lock);

	/*
	 * Calculate mult shift factors for converting
	 * cycles to nanoseconds with 2 year overflow requirement
	 */
	clocks_calc_mult_shift(&ps_dev->mult, &ps_dev->shift, timer_freq_hz,
			       NSEC_PER_SEC, 63072000);

	ret |= init_stat_node(pdev, "SOC_STATS",
			      (char **)&ps_dev->soc_stats_base,
			      sizeof(struct soc_stats));
	ret |= init_stat_node(pdev, "CORE_STATS",
			      (char **)&ps_dev->core_stats_base,
			      sizeof(struct core_stats));
	ret |= init_stat_node(pdev, "FVP_STATS",
			      (char **)&ps_dev->fvp_stats_base,
			      sizeof(struct fvp_stats));
	ret |= init_stat_node(pdev, "PMU_STATS",
			      (char **)&ps_dev->pmu_stats_base,
			      sizeof(struct pmu_stats));
	ret |= init_stat_node(pdev, "LAT_STATS",
			      (char **)&ps_dev->latency_stats_base,
			      sizeof(struct latency_stats));
	ret |= init_pd_stat_node(ps_dev);

	if (ret != 0)
		dev_err(&pdev->dev,
			"Failed to register one or more attributes\n");

	platform_set_drvdata(pdev, ps_dev);

	ret = devm_device_add_groups(&pdev->dev, power_stats_groups);
	if (ret)
		dev_err(&pdev->dev, "Failed to add device groups\n");

	return 0;
}

static int power_stats_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id power_stats_match[] = {
	{
		.compatible = "google,power-stats",
	},
	{},
};
MODULE_DEVICE_TABLE(of, power_stats_match);

static struct platform_driver power_stats_dev = {
	.probe	= power_stats_probe,
	.remove	= power_stats_remove,
	.driver	= 	{
		.name	= "power_stats",
		.owner	= THIS_MODULE,
		.of_match_table = power_stats_match,
	},
};

static int __init power_stats_init(void)
{
	return platform_driver_register(&power_stats_dev);
}

static void __exit power_stats_exit(void)
{
	platform_driver_unregister(&power_stats_dev);
}

module_init(power_stats_init);
module_exit(power_stats_exit);

MODULE_AUTHOR("Benjamin Schwartz <bsschwar@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("APM power stats collection");
