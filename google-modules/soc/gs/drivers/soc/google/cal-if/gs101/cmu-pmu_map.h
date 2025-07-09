/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines CMU_PMU mapping table.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __CMU_PMU_MAP_H__
#define __CMU_PMU_MAP_H__

struct cmu_pmu {
	unsigned int cmu;
	char pmu[20];
};

#endif
