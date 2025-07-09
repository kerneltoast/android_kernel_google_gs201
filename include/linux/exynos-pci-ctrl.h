// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe driver header file for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 */

#ifndef __EXYNOS_PCIE_CTRL_H
#define __EXYNOS_PCIE_CTRL_H

/* PCIe L1SS Control ID */
#define PCIE_L1SS_CTRL_ARGOS	(0x1 << 0)
#define PCIE_L1SS_CTRL_BOOT	(0x1 << 1)
#define PCIE_L1SS_CTRL_CAMERA	(0x1 << 2)
#define PCIE_L1SS_CTRL_MODEM_IF	(0x1 << 3)
#define PCIE_L1SS_CTRL_WIFI	(0x1 << 4)
#define PCIE_L1SS_CTRL_SPEED	(0x1 << 30)
#define PCIE_L1SS_CTRL_TEST	(0x1 << 31)

#if IS_ENABLED(CONFIG_PCI_EXYNOS_GS)
extern int exynos_pcie_l1ss_ctrl(int enable, int id);
extern int exynos_pcie_rc_l1ss_ctrl(int enable, int id, int ch_num);
#endif

#endif
