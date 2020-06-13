/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ACPM_DVFS_H
#define __ACPM_DVFS_H

int exynos_acpm_set_rate(unsigned int id, unsigned long rate);
int exynos_acpm_set_init_freq(unsigned int dfs_id, unsigned long freq);
unsigned long exynos_acpm_get_rate(unsigned int id, unsigned long dbg_val);
int exynos_acpm_set_policy(unsigned int id, unsigned long policy);

#endif /* __ACPM_DVFS_H */
