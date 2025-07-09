/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __ISP_COOLING_H__
#define __ISP_COOLING_H__

#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/cpumask.h>

#define ISP_FPS_INVALID     ~1

#if IS_ENABLED(CONFIG_ISP_THERMAL)

#define ISP_FPS_ENTRY_INVALID	(~0)
#define ISP_FPS_TABLE_END	(~1)

struct isp_fps_table {
	unsigned int	flags;
	unsigned int	driver_data; /* driver specific data, not used by core */
	unsigned int	fps;
};

static inline bool isp_fps_next_valid(struct isp_fps_table **pos)
{
	while ((*pos)->fps != ISP_FPS_TABLE_END) {
		if ((*pos)->fps != ISP_FPS_ENTRY_INVALID)
			return true;

		(*pos)++;
	}
	return false;
}

/*
 * isp_fps_for_each_entry -	iterate over a cpufreq_frequency_table
 * @pos:	the cpufreq_frequency_table * to use as a loop cursor.
 * @table:	the cpufreq_frequency_table * to iterate over.
 */

#define isp_fps_for_each_entry(pos, table)	\
	for (pos = table; pos->fps != ISP_FPS_TABLE_END; pos++)

/*
 * isp_fps_for_each_valid_entry -     iterate over a cpufreq_frequency_table
 *	excluding CPUFREQ_ENTRY_INVALID frequencies.
 * @pos:        the cpufreq_frequency_table * to use as a loop cursor.
 * @table:      the cpufreq_frequency_table * to iterate over.
 */

#define isp_fps_for_each_valid_entry(pos, table)	\
	for (pos = table; isp_fps_next_valid(&pos); pos++)

int exynos_isp_cooling_init(void);
#else
static inline int exynos_isp_cooling_init(void)
{
	return 0;
}
#endif	/* CONFIG_ISP_THERMAL */

#endif /* __ISP_COOLING_H__ */
