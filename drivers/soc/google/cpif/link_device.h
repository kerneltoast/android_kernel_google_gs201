/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINK_DEVICE_H__
#define __LINK_DEVICE_H__

#include <linux/types.h>

#include "link_device_memory.h"
#include "modem_toe_device.h"

bool check_mem_link_tx_pending(struct mem_link_device *mld);
irqreturn_t shmem_tx_state_handler(int irq, void *data);
irqreturn_t shmem_irq_handler(int irq, void *data);

#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
bool shmem_ap2cp_write_clatinfo(struct mem_link_device *mld, struct clat_info *clat);
#endif

#endif /* end of __LINK_DEVICE_H__ */

