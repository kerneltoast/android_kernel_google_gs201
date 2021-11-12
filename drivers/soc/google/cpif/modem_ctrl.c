// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#include "modem_prj.h"
#include "modem_utils.h"

void modem_ctrl_set_kerneltime(struct modem_ctl *mc)
{
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	struct utc_time t;

	get_utc_time(&t);
	mif_info("time = %d.%06d\n", t.sec + (t.min * 60), t.us);

	if (mld->ap2cp_kerneltime_sec.type == DRAM_V2) {
		set_ctrl_msg(&mld->ap2cp_kerneltime_sec, t.sec + (t.min * 60));
		set_ctrl_msg(&mld->ap2cp_kerneltime_usec, t.us);
	} else {
		update_ctrl_msg(&mld->ap2cp_kerneltime, t.sec + (t.min * 60),
				modem->sbi_ap2cp_kerneltime_sec_mask,
				modem->sbi_ap2cp_kerneltime_sec_pos);
		update_ctrl_msg(&mld->ap2cp_kerneltime, t.us,
				modem->sbi_ap2cp_kerneltime_usec_mask,
				modem->sbi_ap2cp_kerneltime_usec_pos);
	}
}

int modem_ctrl_check_offset_data(struct modem_ctl *mc)
{
	struct modem_data *modem = mc->mdm_data;
	struct mem_link_device *mld = modem->mld;
	struct link_device *ld = get_current_link(mc->iod);
	u32 value;

	if (modem->offset_cmsg_offset) {
		value = ioread32(mld->cmsg_offset);
		if (modem->cmsg_offset !=  value) {
			mif_err("ERR: cmsg_offset was damaged: 0x%X (expected: 0x%X)\n",
					value, modem->cmsg_offset);
			goto data_error;
		}
	}

	if (modem->offset_srinfo_offset) {
		value = ioread32(mld->srinfo_offset);
		if (modem->srinfo_offset != value) {
			mif_err("ERR: srinfo_offset was damaged: 0x%X (expected: 0x%X)\n",
					value, modem->srinfo_offset);
			goto data_error;
		}
	}

	if (modem->offset_clk_table_offset) {
		value = ioread32(mld->clk_table_offset);
		if (modem->clk_table_offset != value) {
			mif_err("ERR: clk_table_offset was damaged: 0x%X (expected: 0x%X)\n",
					value, modem->clk_table_offset);
			goto data_error;
		}
	}

	if (modem->offset_buff_desc_offset) {
		value = ioread32(mld->buff_desc_offset);
		if (modem->buff_desc_offset != value) {
			mif_err("ERR: buff_desc_offset was damaged: 0x%X (expected: 0x%X)\n",
					value, modem->buff_desc_offset);
			goto data_error;
		}
	}

	if (ld->capability_check && modem->offset_capability_offset) {
		value = ioread32(mld->capability_offset);
		if (modem->capability_offset != value) {
			mif_err("ERR: capability_offset was damaged: 0x%X (expected: 0x%X)\n",
					value, modem->capability_offset);
			goto data_error;
		}
	}

	mif_info("offset data is ok\n");
	return 0;

data_error:
	mif_err("offset data is damaged\n");
	panic("CPIF shmem offset data is damaged\n");
	return -EFAULT;
}

/* change the modem state & notify io devices about this change */
void change_modem_state(struct modem_ctl *mc, enum modem_state state)
{
	enum modem_state old_state;
	struct io_device *iod;
	unsigned long flags;

	spin_lock_irqsave(&mc->lock, flags);
	old_state = mc->phone_state;
	if (state == old_state) {
		spin_unlock_irqrestore(&mc->lock, flags);
		return; /* no need to wakeup */
	}
	mc->phone_state = state;
	spin_unlock_irqrestore(&mc->lock, flags);

	mif_info("%s->state changed (%s -> %s)\n", mc->name,
		cp_state_str(old_state), cp_state_str(state));

	list_for_each_entry(iod, &mc->modem_state_notify_list, list) {
		if (atomic_read(&iod->opened) > 0)
			wake_up(&iod->wq);
	}
}
