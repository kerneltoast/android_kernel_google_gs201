// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef __PCIE_EXYNOS_RC_DBG_H
#define __PCIE_EXYNOS_RC_DBG_H
int exynos_pcie_dbg_unit_test(struct device *dev, struct exynos_pcie *exynos_pcie);
int exynos_pcie_dbg_link_test(struct device *dev, struct exynos_pcie *exynos_pcie, int enable);
#endif
