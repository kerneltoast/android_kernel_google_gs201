/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos BCM Debugging DT support
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __EXYNOS_BCM_DBG_DT_H_
#define __EXYNOS_BCM_DBG_DT_H_

int exynos_bcm_dbg_parse_dt(struct device_node *np,
				struct exynos_bcm_dbg_data *data);
#endif	/* __EXYNOS_BCM_DBG_DT_H_ */
