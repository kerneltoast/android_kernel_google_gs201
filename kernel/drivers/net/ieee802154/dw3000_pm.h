/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_PM_H
#define __DW3000_PM_H

#include <linux/pm_qos.h>
#include <linux/version.h>

#include "dw3000.h"

#if (KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE)
#define PM_QOS_RESUME_LATENCY_NO_CONSTRAINT S32_MAX
#endif

extern int dw3000_qos_latency;

static inline void dw3000_pm_qos_add_request(struct dw3000 *dw, int latency)
{
#if (KERNEL_VERSION(5, 7, 0) <= LINUX_VERSION_CODE)
	cpu_latency_qos_add_request(&dw->pm_qos_req, latency);
#endif
}

static inline void dw3000_pm_qos_update_request(struct dw3000 *dw, int latency)
{
#if (KERNEL_VERSION(5, 7, 0) <= LINUX_VERSION_CODE)
	cpu_latency_qos_update_request(&dw->pm_qos_req, latency);
#endif
}

static inline void dw3000_pm_qos_remove_request(struct dw3000 *dw)
{
#if (KERNEL_VERSION(5, 7, 0) <= LINUX_VERSION_CODE)
	cpu_latency_qos_remove_request(&dw->pm_qos_req);
#endif
}

#endif /* __DW3000_PM_H  */
