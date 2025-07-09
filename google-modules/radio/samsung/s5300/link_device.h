/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINK_DEVICE_H__
#define __LINK_DEVICE_H__

#include <linux/types.h>

#include "link_device_memory.h"
#include "modem_toe_device.h"

static inline bool ipc_active(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	if (unlikely(!cp_online(mc))) {
		mif_err("%s<->%s: %s.state %s != ONLINE <%ps>\n",
			ld->name, mc->name, mc->name, mc_state(mc), CALLER);
		return false;
	}

	if (mld->dpram_magic) {
		unsigned int magic = ioread32(mld->legacy_link_dev.magic);
		unsigned int mem_access = ioread32(mld->legacy_link_dev.mem_access);

		if (magic != ld->magic_ipc || mem_access != 1) {
			mif_err("%s<->%s: ERR! magic:0x%X access:%d <%ps>\n",
				ld->name, mc->name, magic, mem_access, CALLER);
			return false;
		}
	}

	if (atomic_read(&mld->forced_cp_crash)) {
		mif_err("%s<->%s: ERR! forced_cp_crash:%d <%ps>\n",
			ld->name, mc->name, atomic_read(&mld->forced_cp_crash),
			CALLER);
		return false;
	}

	return true;
}

bool check_mem_link_tx_pending(struct mem_link_device *mld);
irqreturn_t shmem_tx_state_handler(int irq, void *data);
irqreturn_t shmem_irq_handler(int irq, void *data);

#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
bool shmem_ap2cp_write_clatinfo(struct mem_link_device *mld, struct clat_info *clat);
#endif

#endif /* end of __LINK_DEVICE_H__ */

