/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINK_DEVICE_H__
#define __LINK_DEVICE_H__

#include <linux/types.h>

#include "link_device_memory.h"

bool check_mem_link_tx_pending(struct mem_link_device *mld);

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
int request_pcie_msi_int(struct link_device *ld,
				struct platform_device *pdev);
#endif

#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
void shmem_pr_sbdcplog(struct timer_list *t);
#endif

#endif /* end of __LINK_DEVICE_H__ */

