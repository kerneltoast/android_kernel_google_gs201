#ifndef __ACPM_DVFS_H__
#define __ACPM_DVFS_H__

#include <linux/of.h>

struct acpm_dvfs {
	struct device_node *device_node;
	struct device *device;
	unsigned int ch_num;
	unsigned int size;

	unsigned int cpu_len;
	unsigned int gpu_len;
	unsigned int *cpu_coldtemp;
	unsigned int *gpu_coldtemp;
	struct notifier_block cpu_tmu_notifier;
	struct notifier_block gpu_tmu_notifier;

	unsigned int async_ch_num;
	unsigned int async_buffersize;
};

#define FREQ_REQ        0
#define FREQ_GET        1
#define MARGIN_REQ      2
#define COLDTEMP_REQ    3
#define POLICY_REQ      4

#define SET_INIT_FREQ	3

#if IS_ENABLED(CONFIG_ACPM_DVFS)
int exynos_acpm_dvfs_init(void);

extern int exynos_acpm_set_rate(unsigned int id, unsigned long rate);
extern int exynos_acpm_set_init_freq(unsigned int dfs_id, unsigned long freq);
extern int exynos_acpm_get_rate(unsigned int id, unsigned long dbg_val);
extern void exynos_acpm_set_device(struct device *device);
extern int exynos_acpm_set_volt_margin(unsigned int id, int volt);
extern int exynos_acpm_set_cold_temp(unsigned int id, bool is_cold_temp);
extern int exynos_acpm_set_policy(unsigned int id, unsigned long policy);
extern bool exynos_acpm_async_dvfs_enabled(void);
#else
static inline int exynos_acpm_dvfs_init(void)
{
	return 0;
}
static inline int exynos_acpm_set_rate(unsigned int id, unsigned long rate)
{
	return 0;
}

static inline int exynos_acpm_set_init_freq(unsigned int dfs_id, unsigned long freq)
{
	return 0;
}

static inline int exynos_acpm_get_rate(unsigned int id, unsigned long dbg_val)
{
	return 0UL;
}

static inline void exynos_acpm_set_device(struct device *device)
{
	return;
}
static inline int exynos_acpm_set_volt_margin(unsigned int id, int volt)
{
	return 0;
}

static inline int exynos_acpm_set_cold_temp(unsigned int id, bool is_cold_temp)
{
	return 0;
}

static inline int exynos_acpm_set_policy(unsigned int id, unsigned long policy)
{
	return 0;
}
static bool exynos_acpm_async_dvfs_enabled(void)
{
	return 0;
}
#endif
#endif
