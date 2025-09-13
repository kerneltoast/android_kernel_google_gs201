// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe driver header file for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com
 */

#ifndef __PCI_NOTI_H
#define __PCI_NOTI_H

enum exynos_pcie_event {
	EXYNOS_PCIE_EVENT_INVALID = 0,
	EXYNOS_PCIE_EVENT_LINKDOWN = 0x1,
	EXYNOS_PCIE_EVENT_LINKUP = 0x2,
	EXYNOS_PCIE_EVENT_WAKEUP = 0x4,
	EXYNOS_PCIE_EVENT_WAKE_RECOVERY = 0x8,
	EXYNOS_PCIE_EVENT_NO_ACCESS = 0x10,
	EXYNOS_PCIE_EVENT_CPL_TIMEOUT = 0x20,
	EXYNOS_PCIE_EVENT_LINKDOWN_RECOVERY_FAIL = 0x40,
};

enum exynos_pcie_trigger {
	EXYNOS_PCIE_TRIGGER_CALLBACK,
	EXYNOS_PCIE_TRIGGER_COMPLETION,
};

struct exynos_pcie_notify {
	enum exynos_pcie_event event;
	void *user;
	void *data;
	u32 options;
};

struct exynos_pcie_register_event {
	u32 events;
	void *user;
	enum exynos_pcie_trigger mode;
	void (*callback)(struct exynos_pcie_notify *notify);
	struct exynos_pcie_notify notify;
	struct completion *completion;
	u32 options;
};
#endif
