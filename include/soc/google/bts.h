/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BTS header for Exynos.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_BTS_H_
#define __EXYNOS_BTS_H_

/**
 * struct bts_bw - BTS bandwidth information
 * @name:	name of IP
 * @peak:	IP Peak bandwidth
 * @read:	Average read Bandwidth
 * @write:	Average write Bandwidth
 *
 */
struct bts_bw {
	char *name;
	unsigned int peak;
	unsigned int read;
	unsigned int write;
};

#if IS_ENABLED(CONFIG_EXYNOS_BTS) || IS_ENABLED(CONFIG_EXYNOS_BTS_MODULE)
int bts_get_bwindex(const char *name);
int bts_update_bw(unsigned int index, struct bts_bw bw);
unsigned int bts_get_scenindex(const char *name);
int bts_add_scenario(unsigned int index);
int bts_del_scenario(unsigned int index);

void bts_pd_sync(unsigned int cal_id, int on);

#else /* CONFIG_EXYNOS_BTS */

#define bts_get_bwindex(a) do {} while (0)
#define bts_update_bw(a, b) do {} while (0)
#define bts_get_scenindex(a) do {} while (0)
#define bts_add_scenario(a) do {} while (0)
#define bts_del_scenario(a) do {} while (0)
#define bts_pd_sync(a, b) do {} while (0)

#endif /* CONFIG_EXYNOS_BTS */

#define bts_update_scen(a, b) do {} while (0)
#define exynos_bts_scitoken_setting(a) do {} while (0)

#endif /* __EXYNOS_BTS_H_ */
