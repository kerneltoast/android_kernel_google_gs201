/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#ifndef __MODEM_CTRL_H__
#define __MODEM_CTRL_H__

#define MIF_INIT_TIMEOUT	(15 * HZ)

#if IS_ENABLED(CONFIG_SEC_MODEM_S5100)
struct msi_reg_type {
	u32 msi_data;
	u32 msi_check;
	u32 err_report;
	u32 reserved;
	u32 boot_stage;
	u32 img_addr_lo;
	u32 img_addr_hi;
	u32 img_size;
};
#endif

void modem_ctrl_set_kerneltime(struct modem_ctl *mc);
int modem_ctrl_check_offset_data(struct modem_ctl *mc);
void change_modem_state(struct modem_ctl *mc, enum modem_state state);

#if IS_ENABLED(CONFIG_SEC_MODEM_S5100)
int s5100_force_crash_exit_ext(void);
int s5100_poweron_pcie(struct modem_ctl *mc, bool boot_on);
int s5100_try_gpio_cp_wakeup(struct modem_ctl *mc);
int s5100_set_outbound_atu(struct modem_ctl *mc, struct cp_btl *btl,
			   loff_t *pos, u32 map_size);
#endif

#endif /* __MODEM_CTRL_H__ */
